/**
 ****************************************************************************************************
 * @file        comm_monitor.h
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-02
 * @brief       通信监控和超时管理模块
 ****************************************************************************************************
 * @attention
 * 
 * 功能特性:
 * - 通信超时检测
 * - 自动重试机制
 * - 通信状态监控
 * - 错误恢复策略
 * 
 ****************************************************************************************************
 */

#ifndef __COMM_MONITOR_H
#define __COMM_MONITOR_H

#include "stm32f1xx_hal.h"
#include "error_handler.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================== 配置参数 ======================== */

#define COMM_TIMEOUT_MS                 100         /* 通信超时时间(ms) */
#define COMM_MAX_RETRY                  3           /* 最大重试次数 */
#define COMM_RETRY_DELAY_MS             50          /* 重试间隔(ms) */
#define COMM_HEARTBEAT_INTERVAL_MS      1000        /* 心跳间隔(ms) */

/* ======================== 通信状态枚举 ======================== */

/**
 * @brief 通信状态机
 */
typedef enum {
    COMM_STATE_IDLE = 0,                /* 空闲 */
    COMM_STATE_SENDING,                 /* 发送中 */
    COMM_STATE_WAITING_RESPONSE,        /* 等待响应 */
    COMM_STATE_TIMEOUT,                 /* 超时 */
    COMM_STATE_RETRY,                   /* 重试中 */
    COMM_STATE_ERROR                    /* 错误 */
} comm_state_t;

/**
 * @brief 通信命令结构体
 */
typedef struct {
    uint8_t cmd_id;                     /* 命令ID */
    uint32_t send_tick;                 /* 发送时间戳 */
    uint8_t retry_count;                /* 当前重试次数 */
    comm_state_t state;                 /* 通信状态 */
    bool waiting_response;              /* 是否等待响应 */
} comm_cmd_t;

/**
 * @brief 通信统计信息
 */
typedef struct {
    uint32_t total_send;                /* 总发送次数 */
    uint32_t timeout_cnt;               /* 超时次数 */
    uint32_t retry_cnt;                 /* 重试次数 */
    uint32_t success_cnt;               /* 成功次数 */
    uint32_t error_cnt;                 /* 错误次数 */
    uint32_t last_response_tick;        /* 最后响应时间戳 */
    uint32_t max_response_time;         /* 最大响应时间(ms) */
} comm_stats_t;

/* ======================== 函数声明 ======================== */

void comm_monitor_init(void);
error_code_t comm_send_with_retry(uint8_t cmd_id, const uint8_t *data, uint16_t len);
void comm_on_response_received(uint8_t cmd_id);
void comm_check_timeout(void);
bool comm_is_idle(void);
comm_state_t comm_get_state(void);
void comm_get_stats(comm_stats_t *stats);
void comm_clear_stats(void);

#endif /* __COMM_MONITOR_H */
