/**
 ****************************************************************************************************
 * @file        log_system.c
 * @author      正点原子团队(ALIENTEK) - Phase 4优化
 * @version     V1.0
 * @date        2025-12-01
 * @brief       统一日志系统实现
 ****************************************************************************************************
 */

#include "log_system.h"

/* 全局日志级别（默认INFO级别） */
log_level_t g_log_level = LOG_LEVEL_INFO;

/**
 * @brief       初始化日志系统
 * @param       level: 初始日志级别
 * @retval      无
 */
void log_system_init(log_level_t level)
{
    g_log_level = level;
}

/**
 * @brief       动态设置日志级别
 * @param       level: 新的日志级别
 * @retval      无
 */
void log_set_level(log_level_t level)
{
    if (level <= LOG_LEVEL_DEBUG)
    {
        g_log_level = level;
    }
}

/**
 * @brief       获取当前日志级别
 * @param       无
 * @retval      当前日志级别
 */
log_level_t log_get_level(void)
{
    return g_log_level;
}
