/**
 ****************************************************************************************************
 * @file        error_handler.h
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-02
 * @brief       生产级错误处理和诊断系统
 ****************************************************************************************************
 * @attention
 * 
 * 功能特性:
 * - 统一错误码定义
 * - 参数验证宏
 * - 错误统计与追踪
 * - 断言机制 (Debug模式)
 * 
 ****************************************************************************************************
 */

#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================== 错误码定义 ======================== */

/**
 * @brief 统一错误码枚举
 * @note  所有函数返回值统一使用此枚举
 */
typedef enum {
    ERR_OK = 0,                 /* 操作成功 */
    ERR_NULL_POINTER,           /* 空指针 */
    ERR_INVALID_PARAM,          /* 无效参数 */
    ERR_OUT_OF_BOUNDS,          /* 数组越界 */
    ERR_BUFFER_OVERFLOW,        /* 缓冲区溢出 */
    ERR_TIMEOUT,                /* 操作超时 */
    ERR_BUSY,                   /* 设备忙 */
    ERR_NOT_READY,              /* 设备未就绪 */
    ERR_HAL_FAILED,             /* HAL库调用失败 */
    ERR_CHECKSUM,               /* 校验错误 */
    ERR_COMMUNICATION,          /* 通信错误 */
    ERR_NO_RESPONSE,            /* 无响应 */
    ERR_OVERFLOW,               /* 数据溢出 */
    ERR_UNDERFLOW,              /* 数据下溢 */
    ERR_NOT_IMPLEMENTED,        /* 功能未实现 */
    ERR_UNKNOWN                 /* 未知错误 */
} error_code_t;

/* ======================== 错误统计 ======================== */

/**
 * @brief 全局错误统计结构体
 */
typedef struct {
    uint32_t null_pointer_errors;
    uint32_t param_errors;
    uint32_t overflow_errors;
    uint32_t timeout_errors;
    uint32_t checksum_errors;
    uint32_t communication_errors;
    uint32_t total_errors;
    uint32_t last_error;            /* 最后一次错误码 */
    uint32_t last_error_tick;       /* 最后一次错误时间戳 */
} error_stats_t;

/* ======================== 参数验证宏 ======================== */

#ifdef NDEBUG
    /* Release模式: 轻量级检查，返回错误码 */
    #define CHECK_NULL_PTR(ptr) \
        do { \
            if ((ptr) == NULL) { \
                error_log(ERR_NULL_POINTER, __LINE__); \
                return ERR_NULL_POINTER; \
            } \
        } while(0)
    
    #define CHECK_PARAM(expr) \
        do { \
            if (!(expr)) { \
                error_log(ERR_INVALID_PARAM, __LINE__); \
                return ERR_INVALID_PARAM; \
            } \
        } while(0)
    
    #define CHECK_BOUNDS(idx, max) \
        do { \
            if ((idx) >= (max)) { \
                error_log(ERR_OUT_OF_BOUNDS, __LINE__); \
                return ERR_OUT_OF_BOUNDS; \
            } \
        } while(0)
        
    #define ASSERT(expr)  ((void)0)  /* Release: 断言禁用 */
    
    /* V3.5 Phase 3: void函数专用宏（无返回值） */
    #define CHECK_PARAM_VOID(expr) \
        do { \
            if (!(expr)) { \
                error_log(ERR_INVALID_PARAM, __LINE__); \
                return; \
            } \
        } while(0)
    
#else
    /* Debug模式: 完整检查，支持断言 */
    #define CHECK_NULL_PTR(ptr) \
        do { \
            if ((ptr) == NULL) { \
                error_assert(__FILE__, __LINE__, "NULL pointer"); \
                return ERR_NULL_POINTER; \
            } \
        } while(0)
    
    #define CHECK_PARAM(expr) \
        do { \
            if (!(expr)) { \
                error_assert(__FILE__, __LINE__, "Invalid parameter: " #expr); \
                return ERR_INVALID_PARAM; \
            } \
        } while(0)
    
    #define CHECK_BOUNDS(idx, max) \
        do { \
            if ((idx) >= (max)) { \
                error_assert(__FILE__, __LINE__, "Array out of bounds"); \
                return ERR_OUT_OF_BOUNDS; \
            } \
        } while(0)
    
    #define ASSERT(expr) \
        do { \
            if (!(expr)) { \
                error_assert(__FILE__, __LINE__, "Assertion failed: " #expr); \
            } \
        } while(0)
    
    /* V3.5 Phase 3: void函数专用宏（无返回值） */
    #define CHECK_PARAM_VOID(expr) \
        do { \
            if (!(expr)) { \
                error_assert(__FILE__, __LINE__, "Invalid parameter: " #expr); \
                return; \
            } \
        } while(0)
    
#endif

/* ======================== 函数声明 ======================== */

void error_handler_init(void);
void error_log(error_code_t error, uint32_t line);
void error_assert(const char *file, uint32_t line, const char *msg);
const char* error_to_string(error_code_t error);
void error_get_stats(error_stats_t *stats);
void error_clear_stats(void);
uint32_t error_get_total_count(void);

#endif /* __ERROR_HANDLER_H */
