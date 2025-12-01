/**
 ******************************************************************************
 * @file    modbus_hal.h
 * @author  STM32_485 Project (Middlewares Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   Modbus硬件抽象层接口（解耦Middlewares与BSP/SYSTEM）
 ******************************************************************************
 * @attention
 * 
 * 架构设计：Middlewares层完全独立
 * - 不直接include BSP层头文件（emm_v5.h）
 * - 不直接include SYSTEM层头文件（usart.h/delay.h）
 * - 通过回调函数与外界交互
 * - App层负责注册具体实现
 * 
 ******************************************************************************
 */

#ifndef __MODBUS_HAL_H
#define __MODBUS_HAL_H

#include <stdint.h>
#include <stdbool.h>

/* ======================== 电机控制回调接口 ======================== */

/**
 * @brief   电机控制命令回调函数指针
 * @note    由App层注册具体实现（调用emm_v5 API）
 */
typedef struct {
    /* 使能控制 */
    void (*motor_enable)(uint8_t addr, bool enable, bool sync);
    
    /* 位置控制 */
    void (*motor_pos_control)(uint8_t addr, uint8_t dir, uint16_t speed, 
                              uint8_t acc, uint32_t pulses, bool relative, bool sync);
    
    /* 速度控制 */
    void (*motor_vel_control)(uint8_t addr, uint8_t dir, uint16_t speed, 
                              uint8_t acc, bool sync);
    
    /* 急停 */
    void (*motor_stop)(uint8_t addr, bool sync);
    
    /* 位置清零 */
    void (*motor_reset_position)(uint8_t addr);
    
    /* 同步运动 */
    void (*motor_sync_motion)(uint8_t addr);
    
    /* 读取电机状态（可选，用于状态轮询） */
    int (*motor_read_status)(uint8_t addr, uint32_t *position, uint16_t *speed);
    
} modbus_motor_callbacks_t;

/* ======================== UART/延时回调接口 ======================== */

/**
 * @brief   UART和延时回调函数指针
 * @note    modbus_rtu当前未使用，保留以备扩展
 */
typedef struct {
    /* UART发送（如需直接控制RS485） */
    int (*uart_transmit)(uint8_t *data, uint16_t len, uint32_t timeout);
    
    /* UART接收 */
    int (*uart_receive)(uint8_t *data, uint16_t len, uint32_t timeout);
    
    /* 延时函数 */
    void (*delay_ms)(uint32_t ms);
    
} modbus_uart_callbacks_t;

/* ======================== 全局回调注册接口 ======================== */

/**
 * @brief       注册电机控制回调函数
 * @param       callbacks: 回调函数结构体指针
 * @retval      0: 成功, -1: 失败
 */
int modbus_register_motor_callbacks(const modbus_motor_callbacks_t *callbacks);

/**
 * @brief       注册UART/延时回调函数（可选）
 * @param       callbacks: 回调函数结构体指针
 * @retval      0: 成功, -1: 失败
 */
int modbus_register_uart_callbacks(const modbus_uart_callbacks_t *callbacks);

/**
 * @brief       获取已注册的电机回调函数
 * @param       无
 * @retval      回调函数结构体指针（内部使用）
 */
const modbus_motor_callbacks_t* modbus_get_motor_callbacks(void);

#endif /* __MODBUS_HAL_H */
