/**
 ****************************************************************************************************
 * @file        emm_state.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V3.2 (架构重构版)
 * @date        2025-12-02
 * @brief       电机通信状态封装（BSP层：仅硬件相关）
 * @license     Copyright (c) 2025, 正点原子
 ****************************************************************************************************
 * @attention
 *
 * 架构定位：BSP层 - 硬件抽象（无业务状态）
 * - V3.2移除电机控制状态（motor_ctrl_state_t） → 迁移到MULTI_MOTOR中间件
 * - BSP层仅保留通信状态（接收缓冲区、帧标志）
 * - 电机位置/速度/缓存管理请使用 multi_motor_manager API
 *
 * 实验平台: 正点原子 M48Z-M3最小系统板STM32F103版
 * 
 ****************************************************************************************************
 */

#ifndef __EMM_STATE_H
#define __EMM_STATE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "usart.h"  /* 引入USART2_REC_LEN定义 */

/* 电机通信状态结构体（BSP层：仅硬件通信相关） */
typedef struct {
    uint8_t rx_buf[USART2_REC_LEN];           /* 原始接收缓冲（FIFO输出） */
    uint8_t cmd_frame[USART2_REC_LEN];        /* 完整命令帧（帧解析后） */
    uint16_t frame_len;                       /* 当前帧长度 */
    volatile uint8_t frame_complete;          /* 帧完成标志（IDLE中断设置） */
} motor_comm_state_t;

/* ========== V3.2架构重构：通信状态API（BSP层） ========== */

void motor_state_init(void);                     /* 初始化通信状态 */
motor_comm_state_t* motor_get_comm_state(void);  /* 获取通信状态结构体 */
uint8_t* motor_get_cmd_frame(void);              /* 获取命令帧缓冲区 */
uint16_t motor_get_frame_len(void);              /* 获取帧长度 */
bool motor_is_frame_complete(void);              /* 检查帧是否完成 */
void motor_clear_frame_flag(void);               /* 清除帧完成标志 */

/* ========== V3.2架构重构：控制状态已移除 ========== */
/* 电机位置/速度/使能状态管理请使用以下Middlewares层API: */
/* - multi_motor_get_info(addr)     // 获取motor_info_t */
/* - multi_motor_update_status()    // 更新所有电机状态 */
/* - multi_motor_enable_batch()     // 批量使能 */

#endif /* __EMM_STATE_H */
