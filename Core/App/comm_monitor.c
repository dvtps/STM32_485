/**
 ****************************************************************************************************
 * @file        comm_monitor.c
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-02
 * @brief       通信监控和超时管理模块实现
 ****************************************************************************************************
 */

#include "comm_monitor.h"
#include "emm_uart.h"
#include "app_config.h"
#include <stdio.h>
#include <string.h>

/* ======================== 私有变量 ======================== */

static comm_cmd_t g_current_cmd = {0};
static comm_stats_t g_comm_stats = {0};
static uint8_t g_retry_buffer[256] = {0};
static uint16_t g_retry_len = 0;

/* ======================== 公共函数实现 ======================== */

/**
 * @brief       通信监控初始化
 * @param       无
 * @retval      无
 */
void comm_monitor_init(void)
{
    memset(&g_current_cmd, 0, sizeof(comm_cmd_t));
    memset(&g_comm_stats, 0, sizeof(comm_stats_t));
    g_current_cmd.state = COMM_STATE_IDLE;
}

/**
 * @brief       带重试的发送函数
 * @param       cmd_id: 命令ID
 * @param       data: 数据指针
 * @param       len: 数据长度
 * @retval      error_code_t 错误码
 */
error_code_t comm_send_with_retry(uint8_t cmd_id, const uint8_t *data, uint16_t len)
{
    CHECK_NULL_PTR(data);
    CHECK_PARAM(len > 0 && len <= 256);
    
    /* 检查是否空闲 */
    if (g_current_cmd.state != COMM_STATE_IDLE)
    {
        return ERR_BUSY;
    }
    
    /* 保存命令信息 */
    g_current_cmd.cmd_id = cmd_id;
    g_current_cmd.retry_count = 0;
    g_current_cmd.state = COMM_STATE_SENDING;
    g_current_cmd.waiting_response = true;
    
    /* 保存数据用于重试 */
    memcpy(g_retry_buffer, data, len);
    g_retry_len = len;
    
    /* 发送数据 */
    uint8_t result = emm_uart_send_with_timeout(data, len, COMM_TIMEOUT_MS);
    
    if (result == 0)
    {
        /* 发送成功，记录时间戳 */
        g_current_cmd.send_tick = HAL_GetTick();
        g_current_cmd.state = COMM_STATE_WAITING_RESPONSE;
        g_comm_stats.total_send++;
        return ERR_OK;
    }
    else
    {
        /* 发送失败 */
        g_current_cmd.state = COMM_STATE_ERROR;
        g_comm_stats.error_cnt++;
        return ERR_COMMUNICATION;
    }
}

/**
 * @brief       响应接收回调
 * @param       cmd_id: 收到的命令ID
 * @retval      无
 * @note        在主循环中调用
 */
void comm_on_response_received(uint8_t cmd_id)
{
    if (g_current_cmd.state != COMM_STATE_WAITING_RESPONSE)
    {
        return;
    }
    
    /* 检查命令ID是否匹配 */
    if (g_current_cmd.cmd_id == cmd_id)
    {
        uint32_t response_time = HAL_GetTick() - g_current_cmd.send_tick;
        
        /* 更新统计 */
        g_comm_stats.success_cnt++;
        g_comm_stats.last_response_tick = HAL_GetTick();
        
        if (response_time > g_comm_stats.max_response_time)
        {
            g_comm_stats.max_response_time = response_time;
        }
        
        /* 返回空闲状态 */
        g_current_cmd.state = COMM_STATE_IDLE;
        g_current_cmd.waiting_response = false;
        
#if DEBUG_ENABLE && DEBUG_UART_ENABLE
        printf("[COMM] Response OK, time=%ums, retry=%d\r\n", 
               (unsigned int)response_time, g_current_cmd.retry_count);
#endif
    }
}

/**
 * @brief       检查通信超时并重试
 * @param       无
 * @retval      无
 * @note        在主循环中周期调用（10ms）
 */
void comm_check_timeout(void)
{
    if (g_current_cmd.state != COMM_STATE_WAITING_RESPONSE)
    {
        return;
    }
    
    /* 检查是否超时 */
    uint32_t elapsed = HAL_GetTick() - g_current_cmd.send_tick;
    if (elapsed < COMM_TIMEOUT_MS)
    {
        return;  /* 未超时 */
    }
    
    /* 超时处理 */
    g_comm_stats.timeout_cnt++;
    g_current_cmd.state = COMM_STATE_TIMEOUT;
    
    /* 检查是否需要重试 */
    if (g_current_cmd.retry_count < COMM_MAX_RETRY)
    {
        g_current_cmd.retry_count++;
        g_comm_stats.retry_cnt++;
        
#if DEBUG_ENABLE && DEBUG_UART_ENABLE
        printf("[COMM] Timeout, retry %d/%d\r\n", 
               g_current_cmd.retry_count, COMM_MAX_RETRY);
#endif
        
        /* 延迟后重试 */
        HAL_Delay(COMM_RETRY_DELAY_MS);
        
        /* 重新发送 */
        uint8_t result = emm_uart_send_with_timeout(g_retry_buffer, g_retry_len, COMM_TIMEOUT_MS);
        
        if (result == 0)
        {
            g_current_cmd.send_tick = HAL_GetTick();
            g_current_cmd.state = COMM_STATE_WAITING_RESPONSE;
        }
        else
        {
            g_current_cmd.state = COMM_STATE_ERROR;
            g_comm_stats.error_cnt++;
        }
    }
    else
    {
        /* 重试次数用尽 */
        g_current_cmd.state = COMM_STATE_ERROR;
        g_current_cmd.waiting_response = false;
        g_comm_stats.error_cnt++;
        
#if DEBUG_ENABLE && DEBUG_UART_ENABLE
        printf("[COMM] Max retry reached, give up\r\n");
#endif
        
        /* 返回空闲状态（允许后续命令） */
        g_current_cmd.state = COMM_STATE_IDLE;
    }
}

/**
 * @brief       检查通信是否空闲
 * @param       无
 * @retval      true: 空闲, false: 忙
 */
bool comm_is_idle(void)
{
    return (g_current_cmd.state == COMM_STATE_IDLE);
}

/**
 * @brief       获取当前通信状态
 * @param       无
 * @retval      当前状态
 */
comm_state_t comm_get_state(void)
{
    return g_current_cmd.state;
}

/**
 * @brief       获取通信统计信息
 * @param       stats: 输出统计数据的指针
 * @retval      无
 */
void comm_get_stats(comm_stats_t *stats)
{
    if (stats == NULL) return;
    
    __disable_irq();
    memcpy(stats, (const void*)&g_comm_stats, sizeof(comm_stats_t));
    __enable_irq();
}

/**
 * @brief       清除通信统计
 * @param       无
 * @retval      无
 */
void comm_clear_stats(void)
{
    __disable_irq();
    memset(&g_comm_stats, 0, sizeof(comm_stats_t));
    __enable_irq();
}
