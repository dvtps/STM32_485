/**
 ******************************************************************************
 * @file    modbus_adapter.h
 * @author  STM32_485 Project (App Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   Modbus回调适配器头文件
 ******************************************************************************
 */

#ifndef __MODBUS_ADAPTER_H
#define __MODBUS_ADAPTER_H

#include <stdint.h>

/**
 * @brief       初始化Modbus适配器并注册回调
 * @param       无
 * @retval      0: 成功, -1: 失败
 * @note        在main.c初始化阶段调用
 */
int modbus_adapter_init(void);

#endif /* __MODBUS_ADAPTER_H */
