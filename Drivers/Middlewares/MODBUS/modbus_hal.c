/**
 ******************************************************************************
 * @file    modbus_hal.c
 * @author  STM32_485 Project (Middlewares Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   Modbus硬件抽象层接口实现
 ******************************************************************************
 */

#include "modbus_hal.h"
#include <string.h>

/* 全局回调函数存储 */
static modbus_motor_callbacks_t g_motor_callbacks = {0};
static modbus_uart_callbacks_t g_uart_callbacks = {0};

/**
 * @brief       注册电机控制回调函数
 * @param       callbacks: 回调函数结构体指针
 * @retval      0: 成功, -1: 失败
 */
int modbus_register_motor_callbacks(const modbus_motor_callbacks_t *callbacks)
{
    if (callbacks == NULL) {
        return -1;
    }
    
    memcpy(&g_motor_callbacks, callbacks, sizeof(modbus_motor_callbacks_t));
    return 0;
}

/**
 * @brief       注册UART/延时回调函数
 * @param       callbacks: 回调函数结构体指针
 * @retval      0: 成功, -1: 失败
 */
int modbus_register_uart_callbacks(const modbus_uart_callbacks_t *callbacks)
{
    if (callbacks == NULL) {
        return -1;
    }
    
    memcpy(&g_uart_callbacks, callbacks, sizeof(modbus_uart_callbacks_t));
    return 0;
}

/**
 * @brief       获取已注册的电机回调函数
 * @param       无
 * @retval      回调函数结构体指针
 */
const modbus_motor_callbacks_t* modbus_get_motor_callbacks(void)
{
    return &g_motor_callbacks;
}
