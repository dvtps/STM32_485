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
#include "app_tasks.h"
#include "motor_zdt.h"
#include "emm_uart.h"
#include "iwdg.h"
#include "printer_axis.h"
#include "motor_monitor.h"
#include "app_config.h"
#include "logger.h"
#include "realtime_motor.h"
#include "tim2_rt.h"
#include <stdio.h>

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
    /* 阶段1：系统初始化 */
    bsp_system_init();
    bsp_peripheral_init();
    bsp_print_boot_info();
    
    /* 阶段2：功能模块初始化 */
    emm_uart_init();
    
#if REALTIME_MOTOR_ENABLE
    /* ✨ 实时模式：微秒级电机控制 */
    rt_motor_init();
    LOG_RT("实时控制系统已初始化");
    LOG_RT("目标延迟: <10us, 队列: 32条命令");
    
    /* 初始化并启动TIM2定时器（10kHz控制周期） */
    if (tim2_rt_init() == 0)
    {
        tim2_rt_start();
        LOG_TIM("实时定时器已启动 (10kHz, 100us周期)");
    }
    else
    {
        LOG_ERROR("TIM2 初始化失败!\r\n");
    }
#endif
    
    printer_axis_init();    /* 3D打印机3轴初始化 */
    motor_zdt_init();       /* 保留兼容测试代码 */
    
    /* V3.7: 初始化电机监控系统（反馈闭环）*/
    motor_monitor_init();
    motor_monitor_register(0x01);  /* X轴 */
    motor_monitor_register(0x02);  /* Y轴左 */
    motor_monitor_register(0x03);  /* Y轴右 */
    motor_monitor_register(0x04);  /* Z轴 */
    LOG_SYSTEM("电机反馈系统已启用 (V3.7)");
    printf("\r\n正在检测电机在线状态...请稍候\r\n");
    
    app_tasks_init();
    
#if FEATURE_USMART_ENABLE
    usmart_dev.init(72);
    LOG_USMART("串口调试工具已初始化");
    printf("\r\n输入 '?' 或 'help' 获取帮助信息\r\n");
#endif

    HAL_Delay(100);
    
    /* 阶段3：主循环任务调度 */
    while (1)
    {
        app_tasks_run();
        
#if FEATURE_WATCHDOG_ENABLE
        /* 首次喂狗（延迟启动看门狗） */
        static uint8_t iwdg_started = 0;
        if (!iwdg_started)
        {
            iwdg_init(4, 1000);  /* 2秒超时 */
            printf("[IWDG] Watchdog enabled (2s timeout)\r\n");
            iwdg_started = 1;
        }
#endif
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
