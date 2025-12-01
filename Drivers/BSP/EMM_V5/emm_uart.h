/**
 ****************************************************************************************************
 * @file        emm_uart.h
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-01
 * @brief       Emm_V5电机统一RS485通信接口层
 *              封装所有UART发送操作，提供统一的发送接口，简化上层调用
 ****************************************************************************************************
 * @attention
 * 
 * 设计目标:
 * - 统一所有RS485通信，避免直接调用HAL_UART_Transmit分散各处
 * - 提供阻塞/非阻塞/DMA三种发送模式（当前仅实现阻塞模式）
 * - 集中管理发送错误、频率限制、总线冲突等问题
 * - 与emm_v5.c解耦，便于移植到其他硬件平台
 * 
 ****************************************************************************************************
 */

#ifndef __EMM_UART_H
#define __EMM_UART_H

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================== 公共宏定义 ======================== */

/* 发送模式选择 */
#define EMM_UART_MODE_BLOCKING      0  /* 阻塞模式（默认，兼容性最好） */
#define EMM_UART_MODE_INTERRUPT     1  /* 中断模式（非阻塞，需要回调） */
#define EMM_UART_MODE_DMA           2  /* DMA模式（高速大数据量） */

/* 当前使用的发送模式 */
#ifndef EMM_UART_TX_MODE
#define EMM_UART_TX_MODE  EMM_UART_MODE_BLOCKING
#endif

/* 最小发送间隔（ms），防止RS485总线拥堵 */
#define EMM_UART_MIN_TX_INTERVAL_MS  5

/* 默认超时时间（ms） */
#define EMM_UART_DEFAULT_TIMEOUT_MS  100

/* ======================== 公共数据结构 ======================== */

/**
 * @brief  UART发送统计信息
 */
typedef struct {
    uint32_t tx_success_cnt;    /* 成功发送次数 */
    uint32_t tx_error_cnt;      /* 发送失败次数 */
    uint32_t tx_busy_cnt;       /* 发送器忙次数（上次未完成） */
    uint32_t tx_throttle_cnt;   /* 被频率限制拦截次数 */
    uint32_t last_tx_tick;      /* 上次发送时间戳 */
} emm_uart_stats_t;

/* ======================== 公共函数声明 ======================== */

/* 初始化函数 */
void emm_uart_init(void);

/* 核心发送函数 */
uint8_t emm_uart_send(const uint8_t *data, uint16_t len);
uint8_t emm_uart_send_with_timeout(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/* 状态查询函数 */
bool emm_uart_is_busy(void);
void emm_uart_get_stats(emm_uart_stats_t *stats);
void emm_uart_reset_stats(void);

/* 中断模式回调函数（在usart.c中调用） */
void emm_uart_tx_complete_callback(void);
void emm_uart_tx_error_callback(void);

/* 调试辅助函数 */
void emm_uart_print_stats(void);

#endif /* __EMM_UART_H */
