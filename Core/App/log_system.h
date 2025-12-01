/**
 ****************************************************************************************************
 * @file        log_system.h
 * @author      正点原子团队(ALIENTEK) - Phase 4优化
 * @version     V1.0
 * @date        2025-12-01
 * @brief       统一日志系统（分级控制）
 ****************************************************************************************************
 * @attention
 * 
 * 功能说明:
 * - 统一管理LOG_ERROR/WARN/INFO/DEBUG宏
 * - 支持运行时动态调整日志级别
 * - 减少Flash占用（条件编译）
 * 
 ****************************************************************************************************
 */

#ifndef __LOG_SYSTEM_H
#define __LOG_SYSTEM_H

#include <stdio.h>
#include <stdint.h>

/* 日志级别枚举 */
typedef enum {
    LOG_LEVEL_NONE = 0,         /* 禁用所有日志 */
    LOG_LEVEL_ERROR,            /* 仅错误 */
    LOG_LEVEL_WARN,             /* 错误+警告 */
    LOG_LEVEL_INFO,             /* 错误+警告+信息 */
    LOG_LEVEL_DEBUG             /* 所有日志 */
} log_level_t;

/* 全局日志级别变量 */
extern log_level_t g_log_level;

/* 统一日志宏（编译时条件检查 + 运行时级别过滤） */
#if 1  /* 总开关，可设为0完全禁用日志节省Flash */

#define LOG_ERROR(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_ERROR) \
            printf("[ERROR] " fmt, ##__VA_ARGS__); \
    } while(0)

#define LOG_WARN(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_WARN) \
            printf("[WARN] " fmt, ##__VA_ARGS__); \
    } while(0)

#define LOG_INFO(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_INFO) \
            printf("[INFO] " fmt, ##__VA_ARGS__); \
    } while(0)

#define LOG_DEBUG(fmt, ...) \
    do { \
        if (g_log_level >= LOG_LEVEL_DEBUG) \
            printf("[DEBUG] " fmt, ##__VA_ARGS__); \
    } while(0)

#else  /* 日志完全禁用版本 */
#define LOG_ERROR(fmt, ...)  ((void)0)
#define LOG_WARN(fmt, ...)   ((void)0)
#define LOG_INFO(fmt, ...)   ((void)0)
#define LOG_DEBUG(fmt, ...)  ((void)0)
#endif

/* 函数声明 */
void log_system_init(log_level_t level);        /* 初始化日志系统 */
void log_set_level(log_level_t level);          /* 动态设置日志级别 */
log_level_t log_get_level(void);                /* 获取当前日志级别 */

#endif
