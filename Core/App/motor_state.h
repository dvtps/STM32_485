/**
 ****************************************************************************************************
 * @file        motor_state.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-12-01
 * @brief       电机状态封装模块（提高代码可维护性）
 * @license     Copyright (c) 2025, 正点原子
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台: 正点原子 M48Z-M3最小系统板STM32F103版
 * 功能说明:
 * - 将分散的全局变量封装为结构体
 * - 提供统一的状态访问接口
 * - 便于多电机扩展
 * 
 ****************************************************************************************************
 */

#ifndef __MOTOR_STATE_H
#define __MOTOR_STATE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "usart.h"  /* 引入USART2_REC_LEN定义 */

/* 电机通信状态结构体 */
typedef struct {
    uint8_t rx_buf[USART2_REC_LEN];           /* 原始接收缓冲（FIFO输出） */
    uint8_t cmd_frame[USART2_REC_LEN];        /* 完整命令帧（帧解析后） */
    uint16_t frame_len;                       /* 当前帧长度 */
    volatile uint8_t frame_complete;          /* 帧完成标志（IDLE中断设置） */
} motor_comm_state_t;

/* 电机控制状态结构体 (Phase 3扩展缓存) */
typedef struct {
    uint8_t addr;                             /* 电机地址 */
    bool enabled;                             /* 使能状态 */
    int32_t target_pos;                       /* 目标位置（脉冲） */
    int32_t current_pos;                      /* 当前位置（脉冲） */
    uint16_t target_vel;                      /* 目标速度（RPM） */
    uint16_t current_vel;                     /* 当前速度（RPM） */
    uint8_t error_flags;                      /* 错误标志位 */
    uint8_t status_flags;                     /* 状态标志: bit0=使能, bit1=到位, bit2=堵转 */
    uint32_t last_update_tick;                /* 最后更新时间戳(HAL_GetTick) */
    bool cache_valid;                         /* 缓存有效标志 */
} motor_ctrl_state_t;

#define MOTOR_CACHE_TIMEOUT_MS  100           /* 缓存超时100ms */

/* 初始化电机状态模块 */
void motor_state_init(void);

/* 通信状态访问接口 */
motor_comm_state_t* motor_get_comm_state(void);
uint8_t* motor_get_cmd_frame(void);
uint16_t motor_get_frame_len(void);
bool motor_is_frame_complete(void);
void motor_clear_frame_flag(void);

/* 控制状态访问接口 */
motor_ctrl_state_t* motor_get_ctrl_state(uint8_t addr);
void motor_update_position(uint8_t addr, int32_t pos);
void motor_update_status(uint8_t addr, uint8_t flags);       /* 更新状态标志 - Phase 3新增 */
bool motor_is_cache_valid(uint8_t addr);                     /* 检查缓存是否有效 - Phase 3新增 */
void motor_invalidate_cache(uint8_t addr);                   /* 失效缓存 - Phase 3新增 */

#endif
