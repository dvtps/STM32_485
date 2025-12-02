/**
 ******************************************************************************
 * @file    motor_state.h
 * @author  GitHub Copilot (Claude Sonnet 4.5)
 * @version V3.5
 * @date    2025-12-02
 * @brief   多电机状态数据结构定义
 ******************************************************************************
 */

#ifndef __MOTOR_STATE_H
#define __MOTOR_STATE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 最大电机数量 */
#define MAX_MOTOR_COUNT             8

/* 健康度阈值 */
#define MOTOR_HEALTH_THRESHOLD      20

/* 超时阈值(ms) */
#define MOTOR_RESPONSE_TIMEOUT_MS   500

/* 自动恢复间隔(ms) */
#define MOTOR_AUTO_RECOVER_INTERVAL_MS  30000

/* 状态更新间隔(ms) */
#define MOTOR_STATUS_UPDATE_INTERVAL_MS 100

/* 电机在线状态 */
typedef enum {
    MOTOR_STATUS_OFFLINE = 0,
    MOTOR_STATUS_ONLINE  = 1,
    MOTOR_STATUS_ERROR   = 2,
} motor_online_status_t;

/* 电机错误类型 */
typedef enum {
    MOTOR_ERROR_NONE        = 0x00,
    MOTOR_ERROR_TIMEOUT     = 0x01,
    MOTOR_ERROR_CRC         = 0x02,
    MOTOR_ERROR_OVERLOAD    = 0x04,
    MOTOR_ERROR_STALL       = 0x08,
    MOTOR_ERROR_OVERVOLTAGE = 0x10,
    MOTOR_ERROR_UNDERVOLTAGE= 0x20,
} motor_error_type_t;

/* 单个电机状态结构体(64字节) */
typedef struct {
    /* 基础信息 (8字节) */
    uint8_t  address;
    bool     enabled;
    motor_online_status_t online;
    uint8_t  health;
    uint16_t error_flags;
    uint8_t  reserved1[2];
    
    /* 运动状态 (16字节) */
    int32_t  position;
    int16_t  velocity;
    uint8_t  direction;
    uint8_t  is_moving;
    uint16_t target_speed;
    uint8_t  acceleration;
    uint8_t  reserved2[5];
    
    /* 电气参数 (8字节) */
    uint16_t vbus;
    int16_t  current;
    uint16_t temperature;
    uint8_t  reserved3[2];
    
    /* 统计信息 (16字节) */
    uint32_t total_commands;
    uint32_t error_count;
    uint32_t last_response_tick;
    uint32_t last_error_tick;
    
    /* 预留扩展 (16字节) */
    uint8_t  reserved[16];
} motor_state_t;

/* 电机运动参数 */
typedef struct {
    uint8_t  address;
    uint8_t  direction;
    uint16_t speed;
    uint8_t  acceleration;
    uint32_t pulses;
    bool     is_relative;
} motor_move_t;

/* 多电机管理器 */
typedef struct {
    motor_state_t *motors[MAX_MOTOR_COUNT];
    uint8_t motor_count;
    uint32_t last_scan_tick;
    uint32_t last_update_tick;
    uint8_t scan_errors;
    bool initialized;
} motor_manager_t;

#endif /* __MOTOR_STATE_H */
