/**
 ****************************************************************************************************
 * @file        emm_uart.c
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-01
 * @brief       Emm_V5电机统一RS485通信接口层实现
 ****************************************************************************************************
 */

#include "emm_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "error_handler.h"  /* V3.5 Phase 3: 参数验证宏 */

/* ======================== 私有变量 ======================== */

static emm_uart_stats_t g_stats = {0};        /* 统计信息 */
static volatile bool g_tx_busy = false;       /* 发送忙标志 */

/* ======================== 公共函数实现 ======================== */

/**
 * @brief       初始化EMM UART通信模块
 * @param       无
 * @retval      无
 * @note        在usart_init()之后调用
 */
void emm_uart_init(void)
{
    memset(&g_stats, 0, sizeof(emm_uart_stats_t));
    g_tx_busy = false;
}

/**
 * @brief       发送数据到RS485总线（使用默认超时）
 * @param       data: 数据缓冲区指针
 * @param       len: 数据长度
 * @retval      0: 成功, 1: 失败, 2: 发送太频繁
 */
uint8_t emm_uart_send(const uint8_t *data, uint16_t len)
{
    return emm_uart_send_with_timeout(data, len, EMM_UART_DEFAULT_TIMEOUT_MS);
}

/**
 * @brief       发送数据到RS485总线（指定超时时间）
 * @param       data: 数据缓冲区指针
 * @param       len: 数据长度
 * @param       timeout_ms: 超时时间（毫秒）
 * @retval      0: 成功, 1: 失败, 2: 发送太频繁
 */
uint8_t emm_uart_send_with_timeout(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    uint32_t current_tick = HAL_GetTick();
    
    /* V3.5 Phase 3: 参数验证 */
    if (data == NULL || len == 0 || len > 256)
    {
        g_stats.tx_error_cnt++;
        return 1;  /* 参数无效：空指针、零长度或超长 */
    }
    
    if (timeout_ms == 0 || timeout_ms > 10000)
    {
        timeout_ms = EMM_UART_DEFAULT_TIMEOUT_MS;  /* 超时参数异常，使用默认值 */
    }
    
    /* 频率限制检查 */
    if ((current_tick - g_stats.last_tx_tick) < EMM_UART_MIN_TX_INTERVAL_MS)
    {
        g_stats.tx_throttle_cnt++;
        return 2;  /* 发送太频繁，建议调用者稍后重试 */
    }
    
#if (EMM_UART_TX_MODE == EMM_UART_MODE_BLOCKING)
    /* ========== 阻塞模式发送（默认） ========== */
    status = HAL_UART_Transmit(&g_uart2_handle, (uint8_t*)data, len, timeout_ms);
    
    if (status == HAL_OK)
    {
        g_stats.tx_success_cnt++;
        g_stats.last_tx_tick = current_tick;
        return 0;
    }
    else
    {
        g_stats.tx_error_cnt++;
        return 1;
    }
    
#elif (EMM_UART_TX_MODE == EMM_UART_MODE_INTERRUPT)
    /* ========== 中断模式发送 ========== */
    if (g_tx_busy)
    {
        g_stats.tx_busy_cnt++;
        return 1;  /* 上次发送未完成 */
    }
    
    g_tx_busy = true;
    status = HAL_UART_Transmit_IT(&g_uart2_handle, (uint8_t*)data, len);
    
    if (status == HAL_OK)
    {
        g_stats.tx_success_cnt++;
        g_stats.last_tx_tick = current_tick;
        return 0;
    }
    else
    {
        g_tx_busy = false;
        g_stats.tx_error_cnt++;
        return 1;
    }
    
#elif (EMM_UART_TX_MODE == EMM_UART_MODE_DMA)
    /* ========== DMA模式发送（待实现） ========== */
    if (g_tx_busy)
    {
        g_stats.tx_busy_cnt++;
        return 1;
    }
    
    g_tx_busy = true;
    status = HAL_UART_Transmit_DMA(&g_uart2_handle, (uint8_t*)data, len);
    
    if (status == HAL_OK)
    {
        g_stats.tx_success_cnt++;
        g_stats.last_tx_tick = current_tick;
        return 0;
    }
    else
    {
        g_tx_busy = false;
        g_stats.tx_error_cnt++;
        return 1;
    }
    
#else
    #error "Invalid EMM_UART_TX_MODE configuration!"
#endif
}

/**
 * @brief       检查发送器是否忙
 * @param       无
 * @retval      true: 忙, false: 空闲
 */
bool emm_uart_is_busy(void)
{
    return g_tx_busy;
}

/**
 * @brief       获取统计信息
 * @param       stats: 统计信息结构体指针
 * @retval      无
 */
void emm_uart_get_stats(emm_uart_stats_t *stats)
{
    if (stats != NULL)
    {
        memcpy(stats, &g_stats, sizeof(emm_uart_stats_t));
    }
}

/**
 * @brief       重置统计信息
 * @param       无
 * @retval      无
 */
void emm_uart_reset_stats(void)
{
    g_stats.tx_success_cnt = 0;
    g_stats.tx_error_cnt = 0;
    g_stats.tx_busy_cnt = 0;
    g_stats.tx_throttle_cnt = 0;
    /* 保留 last_tx_tick，避免重置后立即发送触发频率限制 */
}

/**
 * @brief       发送完成回调（中断/DMA模式使用）
 * @param       无
 * @retval      无
 * @note        在HAL_UART_TxCpltCallback中调用
 */
void emm_uart_tx_complete_callback(void)
{
#if (EMM_UART_TX_MODE != EMM_UART_MODE_BLOCKING)
    g_tx_busy = false;
#endif
}

/**
 * @brief       发送错误回调（中断/DMA模式使用）
 * @param       无
 * @retval      无
 * @note        在HAL_UART_ErrorCallback中调用
 */
void emm_uart_tx_error_callback(void)
{
#if (EMM_UART_TX_MODE != EMM_UART_MODE_BLOCKING)
    g_tx_busy = false;
    g_stats.tx_error_cnt++;
#endif
}

/**
 * @brief       打印统计信息（调试用）
 * @param       无
 * @retval      无
 */
void emm_uart_print_stats(void)
{
    printf("\r\n[EMM UART Statistics]\r\n");
    printf("TX Success:   %u\r\n", (unsigned int)g_stats.tx_success_cnt);
    printf("TX Error:     %u\r\n", (unsigned int)g_stats.tx_error_cnt);
    printf("TX Busy:      %u\r\n", (unsigned int)g_stats.tx_busy_cnt);
    printf("TX Throttle:  %u\r\n", (unsigned int)g_stats.tx_throttle_cnt);
    printf("Last TX Tick: %u ms\r\n", (unsigned int)g_stats.last_tx_tick);
    
    /* 计算成功率 */
    uint32_t total = g_stats.tx_success_cnt + g_stats.tx_error_cnt;
    if (total > 0)
    {
        float success_rate = (float)g_stats.tx_success_cnt / total * 100.0f;
        printf("Success Rate: %.2f%%\r\n", success_rate);
    }
}
