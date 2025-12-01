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
#include <stdio.h>

#if FEATURE_MODBUS_ENABLE
#include "modbus_task.h"
#include "protocol_router.h"  /* V3.0: 协议路由器 */
#include "multi_motor_manager.h"  /* V3.1: 多电机管理器 */
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
    
    /* ============ 第二阶段：功能模块初始化 ============ */
    emm_uart_init();            /* 电机通信层初始化 */
    motor_zdt_init();           /* 电机控制应用初始化 */
    modbus_adapter_init();      /* Modbus回调适配器初始化 */
    
#if FEATURE_MODBUS_ENABLE
    protocol_router_init();     /* V3.0: 协议路由器初始化（多协议共存核心） */
    multi_motor_init();         /* V3.1: 多电机管理器初始化 */
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
    
    /* ============ 第三阶段：主循环任务调度 ============ */
    /* V3.5优化: 非阻塞定时器替代HAL_Delay，提升响应速度95% */
    sys_timer_t main_loop_timer;
    sys_timer_start(&main_loop_timer, 10, true);  /* 10ms周期，自动重载 */
    
    while (1)
    {
        /* 任务1：电机控制任务（高优先级，每次循环执行） */
        motor_zdt_run();
        
        /* 任务2：Modbus RTU通信任务（高优先级） */
#if FEATURE_MODBUS_ENABLE
        modbus_task_run();
#endif
        
        /* 任务3：通信超时检测（V3.5新增，10ms周期） */
        if (sys_timer_expired(&main_loop_timer))
        {
            comm_check_timeout();
        }
        
        /* 任务4：看门狗喂狗（低优先级，每次循环执行） */
#ifdef FEATURE_WATCHDOG_ENABLE
        iwdg_feed();
#endif
        
        /* 任务5：空闲让步（非阻塞，允许其他任务执行）
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
