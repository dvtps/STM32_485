/**
 ****************************************************************************************************
 * @file        app_tasks.h
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-03
 * @brief       应用层任务调度（主循环任务封装）
 ****************************************************************************************************
 * @attention   设计目标：
 *              1. 封装主循环所有周期性任务
 *              2. 保持main.c简洁清晰
 *              3. 任务的优先级和周期可配置
 ****************************************************************************************************
 */

#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include <stdint.h>
#include <stdbool.h>

/* 任务初始化 */
void app_tasks_init(void);

/* 任务调度（从main循环调用） */
void app_tasks_run(void);

/* 单独的任务接口（可选，用于USMART调试） */
void task_comm_process(void);       /* 通信帧处理 */
void task_motor_control(void);      /* 电机控制 */
void task_led_heartbeat(void);      /* LED心跳指示 */
void task_comm_monitor(void);       /* 通信监控 */
void task_mem_check(void);          /* 内存检测 */
void task_watchdog_feed(void);      /* 看门狗喂狗 */

#endif /* __APP_TASKS_H */
