/**
 ****************************************************************************************************
 * @file        logger.c
 * @author      正点原子团队(ALIENTEK) - Phase 4优化
 * @version     V2.0
 * @date        2025-12-03
 * @brief       统一日志系统实现（带时间戳）
 ****************************************************************************************************
 */

#include "logger.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

/* 全局日志级别（默认INFO级别） */
log_level_t g_log_level = LOG_LEVEL_INFO;

/* ======================== 时间源配置 ======================== */
/* 时间源选择：0=HAL_GetTick (上电计时), 1=RTC模块 (真实时间) */
#define TIME_SOURCE_RTC  0

#if TIME_SOURCE_RTC
/* TODO: 添加RTC驱动头文件 */
/* #include "ds3231.h" */
#endif

/**
 * @brief       获取当前时间戳（预留RTC接口）
 * @note        TIME_SOURCE_RTC=0: 使用HAL_GetTick (开机后计时)
 *              TIME_SOURCE_RTC=1: 使用RTC模块 (真实日期时间)
 * @retval      时间戳结构体
 */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} timestamp_t;

static timestamp_t get_current_time(void)
{
    timestamp_t ts = {0};
    
#if TIME_SOURCE_RTC
    /* TODO: 从RTC读取真实时间 */
    /* rtc_read_time(&ts.hour, &ts.minute, &ts.second); */
    /* ts.millisecond = HAL_GetTick() % 1000;  // RTC一般只有秒级精度 */
#else
    /* 使用HAL_GetTick计算相对时间 */
    uint32_t tick_ms = HAL_GetTick();
    uint32_t seconds = tick_ms / 1000;
    ts.millisecond = tick_ms % 1000;
    
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;
    
    ts.second = seconds % 60;
    ts.minute = minutes % 60;
    ts.hour = hours % 24;  /* 24小时后循环 */
#endif
    
    return ts;
}

/**
 * @brief       格式化时间戳输出
 * @retval      无
 */
static void print_timestamp(void)
{
    timestamp_t ts = get_current_time();
    printf("[%02u:%02u:%02u.%03u] ", 
           ts.hour, ts.minute, ts.second, ts.millisecond);
}

/**
 * @brief       带时间戳的日志输出（通用接口）
 * @param       module: 模块名称
 * @param       format: 格式化字符串
 * @retval      无
 */
void log_print(const char *module, const char *format, ...)
{
    print_timestamp();
    printf("[%s] ", module);
    
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    
    printf("\r\n");
}

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
