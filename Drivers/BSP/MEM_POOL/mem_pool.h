/**
 ******************************************************************************
 * @file    mem_pool.h
 * @author  STM32_485 Project
 * @version V3.5
 * @date    2025-12-02
 * @brief   对象池内存管理模块 - 消除内存碎片和栈溢出风险
 ******************************************************************************
 * @attention
 * 
 * 本模块实现两种内存池：
 * 1. 帧缓冲池：4个256字节块，用于UART接收帧、Modbus帧、EMM_V5命令帧
 * 2. 电机状态池：8个64字节块，用于多电机状态管理
 * 
 * 特性：
 * - 零碎片：预分配静态内存，无malloc/free
 * - 确定性延迟：O(1)分配/释放时间
 * - 泄漏检测：自动跟踪分配源和时间戳
 * - 统计功能：实时使用率、峰值、失败次数
 * 
 ******************************************************************************
 */

#ifndef __MEM_POOL_H
#define __MEM_POOL_H

#include "sys.h"
#include <stdint.h>
#include <stdbool.h>

/* 配置参数 */
#define MEM_POOL_FRAME_SIZE         256     /**< 帧缓冲块大小（字节） */
#define MEM_POOL_FRAME_COUNT        4       /**< 帧缓冲块数量 */
#define MEM_POOL_MOTOR_STATE_SIZE   64      /**< 电机状态块大小（字节） */
#define MEM_POOL_MOTOR_STATE_COUNT  8       /**< 电机状态块数量 */

#define MEM_POOL_LEAK_TIMEOUT_MS    5000    /**< 泄漏检测超时阈值（毫秒） */

/* 错误码 */
typedef enum {
    MEM_POOL_OK = 0,            /**< 成功 */
    MEM_POOL_ERR_FULL,          /**< 池已满 */
    MEM_POOL_ERR_INVALID_PTR,   /**< 无效指针 */
    MEM_POOL_ERR_DOUBLE_FREE,   /**< 重复释放 */
    MEM_POOL_ERR_LEAK_DETECTED  /**< 检测到泄漏 */
} mem_pool_err_t;

/* 池类型 */
typedef enum {
    MEM_POOL_TYPE_FRAME = 0,    /**< 帧缓冲池 */
    MEM_POOL_TYPE_MOTOR_STATE   /**< 电机状态池 */
} mem_pool_type_t;

/* 统计信息 */
typedef struct {
    uint32_t total_allocs;      /**< 总分配次数 */
    uint32_t total_frees;       /**< 总释放次数 */
    uint32_t current_used;      /**< 当前使用块数 */
    uint32_t peak_used;         /**< 峰值使用块数 */
    uint32_t alloc_failures;    /**< 分配失败次数 */
    uint32_t invalid_frees;     /**< 无效释放次数 */
    uint32_t double_frees;      /**< 重复释放次数 */
    uint32_t leak_warnings;     /**< 泄漏告警次数 */
} mem_pool_stats_t;

/* 块元数据（调试用） */
typedef struct {
    bool in_use;                /**< 是否正在使用 */
    uint32_t alloc_time;        /**< 分配时间戳（毫秒） */
    const char *alloc_source;   /**< 分配源文件名 */
    uint16_t alloc_line;        /**< 分配源行号 */
} mem_pool_block_meta_t;

/* ============================================================================
 * 核心API - 内存分配与释放
 * ============================================================================ */

/**
 * @brief  初始化内存池模块
 * @param  无
 * @retval 无
 */
void mem_pool_init(void);

/**
 * @brief  从帧缓冲池分配内存（带调试信息）
 * @param  file: 调用源文件名（自动传入__FILE__）
 * @param  line: 调用源行号（自动传入__LINE__）
 * @retval 成功返回内存指针，失败返回NULL
 * @note   推荐使用宏 MEM_POOL_ALLOC_FRAME() 代替直接调用
 */
void* mem_pool_alloc_frame_dbg(const char *file, uint16_t line);

/**
 * @brief  从电机状态池分配内存（带调试信息）
 * @param  file: 调用源文件名（自动传入__FILE__）
 * @param  line: 调用源行号（自动传入__LINE__）
 * @retval 成功返回内存指针，失败返回NULL
 * @note   推荐使用宏 MEM_POOL_ALLOC_MOTOR_STATE() 代替直接调用
 */
void* mem_pool_alloc_motor_state_dbg(const char *file, uint16_t line);

/**
 * @brief  释放帧缓冲内存
 * @param  ptr: 要释放的内存指针
 * @retval MEM_POOL_OK: 成功
 *         MEM_POOL_ERR_INVALID_PTR: 无效指针
 *         MEM_POOL_ERR_DOUBLE_FREE: 重复释放
 */
mem_pool_err_t mem_pool_free_frame(void *ptr);

/**
 * @brief  释放电机状态内存
 * @param  ptr: 要释放的内存指针
 * @retval MEM_POOL_OK: 成功
 *         MEM_POOL_ERR_INVALID_PTR: 无效指针
 *         MEM_POOL_ERR_DOUBLE_FREE: 重复释放
 */
mem_pool_err_t mem_pool_free_motor_state(void *ptr);

/* 便捷宏：自动传入调试信息 */
#define MEM_POOL_ALLOC_FRAME()         mem_pool_alloc_frame_dbg(__FILE__, __LINE__)
#define MEM_POOL_ALLOC_MOTOR_STATE()   mem_pool_alloc_motor_state_dbg(__FILE__, __LINE__)

/* ============================================================================
 * 统计与诊断API
 * ============================================================================ */

/**
 * @brief  获取内存池统计信息
 * @param  type: 池类型（帧缓冲/电机状态）
 * @param  stats: 输出统计结构体指针
 * @retval 无
 */
void mem_pool_get_stats(mem_pool_type_t type, mem_pool_stats_t *stats);

/**
 * @brief  检查内存泄漏（扫描超时未释放的块）
 * @param  current_time: 当前时间戳（毫秒，建议使用HAL_GetTick()）
 * @retval 泄漏块数量（0表示无泄漏）
 * @note   建议在主循环中定期调用（例如1秒周期）
 */
uint32_t mem_pool_check_leaks(uint32_t current_time);

/**
 * @brief  打印内存池统计信息到串口（USART1）
 * @param  无
 * @retval 无
 * @note   用于USMART命令 mem_stats()
 */
void mem_pool_print_stats(void);

/**
 * @brief  打印详细的泄漏报告
 * @param  current_time: 当前时间戳（毫秒）
 * @retval 无
 * @note   用于USMART命令 mem_check_leaks()
 */
void mem_pool_print_leak_report(uint32_t current_time);

/**
 * @brief  重置统计计数器（保留当前分配状态）
 * @param  type: 池类型（帧缓冲/电机状态）
 * @retval 无
 */
void mem_pool_reset_stats(mem_pool_type_t type);

/* ============================================================================
 * 内联辅助函数
 * ============================================================================ */

/**
 * @brief  获取帧缓冲池当前使用率（百分比）
 * @param  无
 * @retval 使用率 0-100
 */
static inline uint8_t mem_pool_get_frame_usage_percent(void)
{
    mem_pool_stats_t stats;
    mem_pool_get_stats(MEM_POOL_TYPE_FRAME, &stats);
    return (stats.current_used * 100) / MEM_POOL_FRAME_COUNT;
}

/**
 * @brief  获取电机状态池当前使用率（百分比）
 * @param  无
 * @retval 使用率 0-100
 */
static inline uint8_t mem_pool_get_motor_state_usage_percent(void)
{
    mem_pool_stats_t stats;
    mem_pool_get_stats(MEM_POOL_TYPE_MOTOR_STATE, &stats);
    return (stats.current_used * 100) / MEM_POOL_MOTOR_STATE_COUNT;
}

#endif /* __MEM_POOL_H */
