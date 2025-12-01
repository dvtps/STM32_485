/**
 ****************************************************************************************************
 * @file        error_handler.c
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-02
 * @brief       生产级错误处理和诊断系统实现
 ****************************************************************************************************
 */

#include "error_handler.h"
#include "app_config.h"
#include <stdio.h>
#include <string.h>

/* ======================== 私有变量 ======================== */

static error_stats_t g_error_stats = {0};
static volatile bool g_error_handler_initialized = false;

/* ======================== 公共函数实现 ======================== */

/**
 * @brief       错误处理器初始化
 * @param       无
 * @retval      无
 */
void error_handler_init(void)
{
    memset(&g_error_stats, 0, sizeof(error_stats_t));
    g_error_handler_initialized = true;
}

/**
 * @brief       记录错误
 * @param       error: 错误码
 * @param       line: 行号
 * @retval      无
 * @note        线程安全，可在中断中调用
 */
void error_log(error_code_t error, uint32_t line)
{
    if (!g_error_handler_initialized) return;
    
    /* 禁用中断保护统计数据 */
    __disable_irq();
    
    /* 更新统计 */
    switch (error)
    {
        case ERR_NULL_POINTER:
            g_error_stats.null_pointer_errors++;
            break;
        case ERR_INVALID_PARAM:
            g_error_stats.param_errors++;
            break;
        case ERR_BUFFER_OVERFLOW:
        case ERR_OVERFLOW:
            g_error_stats.overflow_errors++;
            break;
        case ERR_TIMEOUT:
            g_error_stats.timeout_errors++;
            break;
        case ERR_CHECKSUM:
            g_error_stats.checksum_errors++;
            break;
        case ERR_COMMUNICATION:
        case ERR_NO_RESPONSE:
            g_error_stats.communication_errors++;
            break;
        default:
            break;
    }
    
    g_error_stats.total_errors++;
    g_error_stats.last_error = (uint32_t)error;
    g_error_stats.last_error_tick = HAL_GetTick();
    
    __enable_irq();
    
#if DEBUG_ENABLE && DEBUG_UART_ENABLE
    printf("[ERROR] Code=%d, Line=%u, Tick=%u\r\n", 
           (int)error, (unsigned int)line, (unsigned int)g_error_stats.last_error_tick);
#endif
}

/**
 * @brief       断言失败处理 (仅Debug模式)
 * @param       file: 文件名
 * @param       line: 行号
 * @param       msg: 错误消息
 * @retval      无
 * @note        Release模式此函数为空
 */
void error_assert(const char *file, uint32_t line, const char *msg)
{
#ifndef NDEBUG
    /* Debug模式: 打印详细信息并死循环 */
    printf("\r\n!!! ASSERTION FAILED !!!\r\n");
    printf("File: %s\r\n", file ? file : "unknown");
    printf("Line: %u\r\n", (unsigned int)line);
    printf("Message: %s\r\n", msg ? msg : "no message");
    printf("System halted. Press reset button.\r\n");
    
    __disable_irq();
    while(1)
    {
        /* 死循环等待复位 */
    }
#else
    (void)file;
    (void)line;
    (void)msg;
#endif
}

/**
 * @brief       错误码转字符串
 * @param       error: 错误码
 * @retval      错误描述字符串
 */
const char* error_to_string(error_code_t error)
{
    switch (error)
    {
        case ERR_OK:                return "OK";
        case ERR_NULL_POINTER:      return "NULL_POINTER";
        case ERR_INVALID_PARAM:     return "INVALID_PARAM";
        case ERR_OUT_OF_BOUNDS:     return "OUT_OF_BOUNDS";
        case ERR_BUFFER_OVERFLOW:   return "BUFFER_OVERFLOW";
        case ERR_TIMEOUT:           return "TIMEOUT";
        case ERR_BUSY:              return "BUSY";
        case ERR_NOT_READY:         return "NOT_READY";
        case ERR_HAL_FAILED:        return "HAL_FAILED";
        case ERR_CHECKSUM:          return "CHECKSUM";
        case ERR_COMMUNICATION:     return "COMMUNICATION";
        case ERR_NO_RESPONSE:       return "NO_RESPONSE";
        case ERR_OVERFLOW:          return "OVERFLOW";
        case ERR_UNDERFLOW:         return "UNDERFLOW";
        case ERR_NOT_IMPLEMENTED:   return "NOT_IMPLEMENTED";
        default:                    return "UNKNOWN";
    }
}

/**
 * @brief       获取错误统计信息
 * @param       stats: 输出统计数据的指针
 * @retval      无
 */
void error_get_stats(error_stats_t *stats)
{
    if (stats == NULL) return;
    
    __disable_irq();
    memcpy(stats, (const void*)&g_error_stats, sizeof(error_stats_t));
    __enable_irq();
}

/**
 * @brief       清除错误统计
 * @param       无
 * @retval      无
 */
void error_clear_stats(void)
{
    __disable_irq();
    memset(&g_error_stats, 0, sizeof(error_stats_t));
    __enable_irq();
}

/**
 * @brief       获取错误总数
 * @param       无
 * @retval      错误总数
 */
uint32_t error_get_total_count(void)
{
    return g_error_stats.total_errors;
}
