/**
 ****************************************************************************************************
 * @file        app_tasks.c
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-03
 * @brief       应用层任务调度实现
 ****************************************************************************************************
 */

#include "app_tasks.h"
#include "motor_zdt.h"
#include "emm_uart.h"
#include "motor_monitor.h"
#include "sys_timer.h"
#include <stdio.h>

#if FEATURE_WATCHDOG_ENABLE
#include "iwdg.h"
#endif

/* 任务统计 */
static struct {
    uint32_t comm_process_cnt;      /* 通信处理次数 */
    uint32_t motor_control_cnt;     /* 电机控制次数 */
    uint32_t motor_monitor_cnt;     /* 电机监控次数（V3.7）*/
    uint32_t watchdog_feed_cnt;     /* 喂狗次数 */
} g_task_stats = {0};

/**
 * @brief       任务初始化
 * @param       无
 * @retval      无
 */
void app_tasks_init(void)
{
    LOG_TASK("任务系统已初始化");
}

/**
 * @brief       任务1：通信帧处理
 * @param       无
 * @retval      无
 * @note        每次主循环都调用，高优先级
 */
void task_comm_process(void)
{
    emm_uart_process_frame();
    
    /* V3.7: 处理电机监控响应帧 */
    extern uint8_t g_emm_rx_cmd[];
    extern uint16_t g_emm_rx_count;
    extern volatile uint8_t g_emm_frame_complete;
    
    if (g_emm_frame_complete && g_emm_rx_count > 0) {
        /* 处理响应帧（状态变化时会在 motor_monitor.c 中打印）*/
        motor_monitor_process_response(g_emm_rx_cmd, g_emm_rx_count);
        
        /* 清除帧完成标志，允许处理下一帧 */
        g_emm_frame_complete = 0;
    }
    
    g_task_stats.comm_process_cnt++;
}

/**
 * @brief       任务2：电机控制
 * @param       无
 * @retval      无
 * @note        每次主循环都调用，高优先级
 */
void task_motor_control(void)
{
    motor_zdt_run();
    g_task_stats.motor_control_cnt++;
}

/**
 * @brief       任务3：看门狗喂狗
 * @param       无
 * @retval      无
 * @note        每次主循环都调用
 */
void task_watchdog_feed(void)
{
#if FEATURE_WATCHDOG_ENABLE
    iwdg_feed();
    g_task_stats.watchdog_feed_cnt++;
#endif
}

/**
 * @brief       任务4：电机监控（V3.7反馈闭环）
 * @param       无
 * @retval      无
 * @note        非阻塞轮询，执行时间<1ms
 */
void task_motor_monitor(void)
{
    motor_monitor_task();
    g_task_stats.motor_monitor_cnt++;
}

/**
 * @brief       任务调度主函数
 * @param       无
 * @retval      无
 * @note        从main循环调用，统一调度所有任务
 */
void app_tasks_run(void)
{
    task_comm_process();    /* 优先级1: 通信帧处理 */
    task_motor_control();   /* 优先级2: 电机控制 */
    task_motor_monitor();   /* 优先级3: 电机监控（V3.7）*/
    task_watchdog_feed();   /* 优先级4: 看门狗喂狗 */
}
