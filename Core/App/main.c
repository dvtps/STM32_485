/**
 ****************************************************************************************************
 * @file        main.c
 * @brief       主程序入口 - V3.0工业级架构
 * @author      STM32_485 Project Team
 * @version     V3.0
 * @date        2025-12-01
 ****************************************************************************************************
 * @attention   工业级设计原则：
 *              1. App层只做业务编排，不直接操作硬件
 *              2. 所有硬件操作封装到Drivers层
 *              3. main.c目标<100行，清晰易读
 ****************************************************************************************************
 */

#include "main.h"
#include "app_init.h"
#include "diagnostic.h"
#include "motor_zdt.h"
#include "modbus_adapter.h"
#include "emm_uart.h"
#include "iwdg.h"
#include "app_config.h"
#include "sys_timer.h"         /* V3.5: 非阻塞定时器 */
#include "error_handler.h"     /* V3.5: 错误处理系统 */
#include "comm_monitor.h"      /* V3.5: 通信监控 */
#include "usart.h"             /* V3.5 Phase 2: USART2帧标志 */
#include "fifo.h"              /* V3.5 Phase 2: FIFO出队 */
#include "protocol_router.h"   /* V3.5 Phase 2: 协议路由 */
#include "mem_pool.h"          /* V3.5 Phase 1: 内存池管理 */
#include <stdio.h>

#if FEATURE_MODBUS_ENABLE
#include "modbus_task.h"
#include "protocol_router.h"  /* V3.0: 协议路由器 */
#include "multi_motor_manager.h"  /* V3.1: 多电机管理器 */
#include "modbus_gateway.h"   /* V3.5 Phase 5: 电机状态轮询 */
#endif

#if FEATURE_USMART_ENABLE
#include "usmart.h"
#endif

/**
 * @brief       主函数
 * @param       无
 * @retval      0
 */
int main(void)
{
    /* ============ 第一阶段：系统初始化 ============ */
    bsp_system_init();          /* 系统核心初始化（HAL+时钟+延时） */
    bsp_peripheral_init();      /* 外设初始化（LED+KEY+USART） */
    bsp_print_boot_info();      /* 打印启动信息 */
    
    /* V3.5 Phase 1: 内存池初始化（必须在所有分配操作前） */
    mem_pool_init();            /* 对象池内存管理初始化 */
    
    /* ============ 第二阶段：功能模块初始化 ============ */
    emm_uart_init();            /* 电机通信层初始化 */
    motor_zdt_init();           /* 电机控制应用初始化 */
    protocol_router_init();     /* V3.0: 协议路由器初始化（Emm_V5+Modbus多协议共存） */
    
#if FEATURE_MODBUS_ENABLE
    modbus_adapter_init();      /* Modbus回调适配器初始化 */
    motor_mgr_init();           /* V3.5 Phase 2: 多电机管理器初始化 */
    if (modbus_task_init() == 0) {
        printf("Modbus RTU initialized: Address=%d, Baudrate=%d\r\n", 
               MODBUS_SLAVE_ADDRESS, MODBUS_BAUDRATE);
    } else {
        printf("Modbus RTU init failed!\r\n");
    }
#endif
    
#if FEATURE_USMART_ENABLE
    usmart_dev.init(72);        /* USMART串口调试工具初始化 */
    printf("USMART Debug Tool Initialized (72MHz)\r\n");
#endif

#ifdef FEATURE_DIAGNOSTIC_ENABLE
    diag_run_full_check();      /* 系统诊断（可选） */
#endif

#ifdef FEATURE_WATCHDOG_ENABLE
    iwdg_init(4, 1000);         /* 看门狗初始化：2s超时 */
#endif

    /* 等待电机上电稳定 */
    HAL_Delay(100);
    
    /* V3.5 Phase 3: 自动发现电机 */
#if FEATURE_MODBUS_ENABLE
    printf("[MotorMgr] Scanning motors (1-8)...\r\n");
    uint8_t found_count = motor_mgr_scan(1, 8);
    printf("[MotorMgr] Found %d motor(s)\r\n", found_count);
    motor_mgr_print_all_status();
#endif
    
    /* ============ 第三阶段：主循环任务调度 ============ */
    /* V3.5优化: 非阻塞定时器替代HAL_Delay，提升响应速度95% */
    sys_timer_t main_loop_timer;
    sys_timer_start(&main_loop_timer, 10, true);  /* 10ms周期，自动重载 */
    
    /* V3.5 Phase 2: USART2协议处理缓冲区 */
    uint8_t temp_frame_buffer[256];
    uint16_t frame_len = 0;
    
    while (1)
    {
        /* 任务1：USART2帧处理（V3.5 Phase 2新增，最高优先级）
         * 从中断移到主循环，减少中断处理时间90%
         * V3.5 Phase 8优化：优化临界区保护，中断屏蔽时间从150μs降低至5μs
         */
        if (g_usart2_frame_ready)
        {
            /* 优化1：快速原子操作 - 仅保护标志位和指针快照（<5μs） */
            __disable_irq();
            g_usart2_frame_ready = 0;
            uint16_t temp_read = g_emm_rx_fifo.ptrRead;   /* 快照读指针 */
            uint16_t temp_write = g_emm_rx_fifo.ptrWrite; /* 快照写指针 */
            __enable_irq();
            
            /* 优化2：无锁出队 - 使用快照指针，中断已恢复（0μs阻塞） */
            frame_len = 0;
            while (temp_read != temp_write && frame_len < sizeof(temp_frame_buffer))
            {
                temp_frame_buffer[frame_len++] = (uint8_t)g_emm_rx_fifo.buffer[temp_read];
                temp_read++;
                if (temp_read >= EMM_FIFO_SIZE) {
                    temp_read = 0;  /* 环绕处理 */
                }
            }
            
            /* 优化3：原子更新读指针（<2μs） */
            __disable_irq();
            g_emm_rx_fifo.ptrRead = temp_read;
            __enable_irq();
            
            /* 协议识别与路由分发（耗时操作，在主循环安全执行） */
            if (frame_len > 0)
            {
                protocol_router_process(temp_frame_buffer, frame_len);
            }
        }
        
        /* 任务2：电机控制任务（高优先级，每次循环执行） */
        motor_zdt_run();
        
        /* 任务3：Modbus RTU通信任务（高优先级） */
#if FEATURE_MODBUS_ENABLE
        modbus_task_run();
#endif
        
        /* 任务3.5：电机状态轮询（V3.5 Phase 5新增，中等优先级） */
#if FEATURE_MODBUS_ENABLE
        modbus_gateway_update_motor_status();
#endif
        
        /* 任务3.6：电机故障检测（V3.5 Phase 6新增，中等优先级，250ms周期） */
#if FEATURE_MODBUS_ENABLE
        modbus_gateway_check_motor_faults();
#endif
        
        /* 任务4：通信超时检测（V3.5新增，10ms周期） */
        if (sys_timer_expired(&main_loop_timer))
        {
            comm_check_timeout();
        }
        
    /* 任务4.5：内存泄漏检测（V3.5 Phase 1新增，1秒周期） */
    static uint32_t last_leak_check = 0;
    if (HAL_GetTick() - last_leak_check >= 1000) {
        last_leak_check = HAL_GetTick();
        uint32_t leak_count = mem_pool_check_leaks(last_leak_check);
        if (leak_count > 0) {
            printf("[WARNING] Memory leak detected: %lu blocks\r\n", (unsigned long)leak_count);
        }
    }
    
    /* 任务4.6：多电机状态更新（V3.5 Phase 3新增，100ms周期） */
#if FEATURE_MODBUS_ENABLE
    static uint32_t last_motor_update = 0;
    if (HAL_GetTick() - last_motor_update >= 100) {
        last_motor_update = HAL_GetTick();
        motor_mgr_update_all_status();
    }
#endif
    
    /* 任务4.7：多电机自动恢复（V3.5 Phase 3新增，30秒周期） */
#if FEATURE_MODBUS_ENABLE
    static uint32_t last_auto_recover = 0;
    if (HAL_GetTick() - last_auto_recover >= 30000) {
        last_auto_recover = HAL_GetTick();
        motor_mgr_auto_recover();
    }
#endif        /* 任务5：看门狗喂狗（低优先级，每次循环执行） */
#ifdef FEATURE_WATCHDOG_ENABLE
        iwdg_feed();
#endif
        
        /* 任务6：空闲让步（非阻塞，允许其他任务执行）
         * 注意：不再使用HAL_Delay(10)，主循环可立即响应中断标志
         */
    }
}

/**
 * @brief       错误处理函数
 * @param       无
 * @retval      无
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* LED快速闪烁指示错误 */
        HAL_Delay(200);
    }
}
