/**
 ******************************************************************************
 * @file    modbus_task.h
 * @author  STM32_485 Project (App Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   Modbus RTU任务处理模块头文件
 ******************************************************************************
 */

#ifndef __MODBUS_TASK_H
#define __MODBUS_TASK_H

#include <stdint.h>

/**
 * @brief       Modbus任务初始化
 * @param       无
 * @retval      0: 成功, -1: 失败
 * @note        在main.c初始化阶段调用
 */
int modbus_task_init(void);

/**
 * @brief       Modbus任务主循环
 * @param       无
 * @retval      无
 * @note        在main.c主循环中调用，处理Modbus请求
 */
void modbus_task_run(void);

#endif /* __MODBUS_TASK_H */
