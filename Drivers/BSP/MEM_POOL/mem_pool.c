/**
 ******************************************************************************
 * @file    mem_pool.c
 * @author  STM32_485 Project
 * @version V3.5
 * @date    2025-12-02
 * @brief   对象池内存管理模块实现
 ******************************************************************************
 * @attention
 * 
 * 实现原理：
 * 1. 预分配静态数组作为内存池
 * 2. 位图(bitmap)管理空闲块：0=空闲，1=占用
 * 3. 每块附加元数据：分配时间、源文件、行号
 * 4. O(1)分配：遍历位图找第一个0
 * 5. O(1)释放：计算指针偏移，清除位图
 * 
 * 内存布局示例（帧缓冲池）：
 * +---------+---------+---------+---------+
 * | Block 0 | Block 1 | Block 2 | Block 3 |  256B × 4 = 1024B
 * +---------+---------+---------+---------+
 *     ↓         ↓         ↓         ↓
 * [Meta 0] [Meta 1] [Meta 2] [Meta 3]       32B (8B × 4)
 * 
 ******************************************************************************
 */

#include "mem_pool.h"
#include "usart.h"  /* 用于printf输出 */
#include <string.h>

/* ============================================================================
 * 私有数据结构
 * ============================================================================ */

/* 内存池控制块 */
typedef struct {
    uint8_t *pool_base;             /**< 池基地址 */
    mem_pool_block_meta_t *metadata; /**< 元数据数组 */
    uint16_t block_size;            /**< 块大小 */
    uint16_t block_count;           /**< 块数量 */
    mem_pool_stats_t stats;         /**< 统计信息 */
} mem_pool_ctrl_t;

/* 帧缓冲池静态内存 */
static uint8_t s_frame_pool[MEM_POOL_FRAME_SIZE * MEM_POOL_FRAME_COUNT];
static mem_pool_block_meta_t s_frame_meta[MEM_POOL_FRAME_COUNT];
static mem_pool_ctrl_t s_frame_ctrl;

/* 电机状态池静态内存 */
static uint8_t s_motor_state_pool[MEM_POOL_MOTOR_STATE_SIZE * MEM_POOL_MOTOR_STATE_COUNT];
static mem_pool_block_meta_t s_motor_state_meta[MEM_POOL_MOTOR_STATE_COUNT];
static mem_pool_ctrl_t s_motor_state_ctrl;

/* ============================================================================
 * 私有函数声明
 * ============================================================================ */

static void* mem_pool_alloc_internal(mem_pool_ctrl_t *ctrl, const char *file, uint16_t line);
static mem_pool_err_t mem_pool_free_internal(mem_pool_ctrl_t *ctrl, void *ptr);
static int32_t mem_pool_get_block_index(mem_pool_ctrl_t *ctrl, void *ptr);

/* ============================================================================
 * 公共API实现
 * ============================================================================ */

/**
 * @brief  初始化内存池模块
 */
void mem_pool_init(void)
{
    /* 初始化帧缓冲池控制块 */
    s_frame_ctrl.pool_base = s_frame_pool;
    s_frame_ctrl.metadata = s_frame_meta;
    s_frame_ctrl.block_size = MEM_POOL_FRAME_SIZE;
    s_frame_ctrl.block_count = MEM_POOL_FRAME_COUNT;
    memset(&s_frame_ctrl.stats, 0, sizeof(mem_pool_stats_t));
    memset(s_frame_meta, 0, sizeof(s_frame_meta));
    
    /* 初始化电机状态池控制块 */
    s_motor_state_ctrl.pool_base = s_motor_state_pool;
    s_motor_state_ctrl.metadata = s_motor_state_meta;
    s_motor_state_ctrl.block_size = MEM_POOL_MOTOR_STATE_SIZE;
    s_motor_state_ctrl.block_count = MEM_POOL_MOTOR_STATE_COUNT;
    memset(&s_motor_state_ctrl.stats, 0, sizeof(mem_pool_stats_t));
    memset(s_motor_state_meta, 0, sizeof(s_motor_state_meta));
    
    printf("[MEM_POOL] Initialized:\r\n");
    printf("  Frame Pool: %d blocks x %d bytes = %d bytes\r\n", 
           MEM_POOL_FRAME_COUNT, MEM_POOL_FRAME_SIZE, 
           MEM_POOL_FRAME_COUNT * MEM_POOL_FRAME_SIZE);
    printf("  Motor State Pool: %d blocks x %d bytes = %d bytes\r\n", 
           MEM_POOL_MOTOR_STATE_COUNT, MEM_POOL_MOTOR_STATE_SIZE,
           MEM_POOL_MOTOR_STATE_COUNT * MEM_POOL_MOTOR_STATE_SIZE);
}

/**
 * @brief  从帧缓冲池分配内存
 */
void* mem_pool_alloc_frame_dbg(const char *file, uint16_t line)
{
    return mem_pool_alloc_internal(&s_frame_ctrl, file, line);
}

/**
 * @brief  从电机状态池分配内存
 */
void* mem_pool_alloc_motor_state_dbg(const char *file, uint16_t line)
{
    return mem_pool_alloc_internal(&s_motor_state_ctrl, file, line);
}

/**
 * @brief  释放帧缓冲内存
 */
mem_pool_err_t mem_pool_free_frame(void *ptr)
{
    return mem_pool_free_internal(&s_frame_ctrl, ptr);
}

/**
 * @brief  释放电机状态内存
 */
mem_pool_err_t mem_pool_free_motor_state(void *ptr)
{
    return mem_pool_free_internal(&s_motor_state_ctrl, ptr);
}

/**
 * @brief  获取内存池统计信息
 */
void mem_pool_get_stats(mem_pool_type_t type, mem_pool_stats_t *stats)
{
    if (stats == NULL) return;
    
    mem_pool_ctrl_t *ctrl = (type == MEM_POOL_TYPE_FRAME) ? &s_frame_ctrl : &s_motor_state_ctrl;
    *stats = ctrl->stats;
}

/**
 * @brief  检查内存泄漏
 */
uint32_t mem_pool_check_leaks(uint32_t current_time)
{
    uint32_t leak_count = 0;
    
    /* 检查帧缓冲池 */
    for (uint16_t i = 0; i < MEM_POOL_FRAME_COUNT; i++) {
        if (s_frame_meta[i].in_use) {
            uint32_t elapsed = current_time - s_frame_meta[i].alloc_time;
            if (elapsed > MEM_POOL_LEAK_TIMEOUT_MS) {
                leak_count++;
                s_frame_ctrl.stats.leak_warnings++;
            }
        }
    }
    
    /* 检查电机状态池 */
    for (uint16_t i = 0; i < MEM_POOL_MOTOR_STATE_COUNT; i++) {
        if (s_motor_state_meta[i].in_use) {
            uint32_t elapsed = current_time - s_motor_state_meta[i].alloc_time;
            if (elapsed > MEM_POOL_LEAK_TIMEOUT_MS) {
                leak_count++;
                s_motor_state_ctrl.stats.leak_warnings++;
            }
        }
    }
    
    return leak_count;
}

/**
 * @brief  打印内存池统计信息
 */
void mem_pool_print_stats(void)
{
    printf("\r\n========== Memory Pool Statistics ==========\r\n");
    
    /* 帧缓冲池 */
    printf("[Frame Buffer Pool] (%d x %dB = %dB total)\r\n", 
           MEM_POOL_FRAME_COUNT, MEM_POOL_FRAME_SIZE,
           MEM_POOL_FRAME_COUNT * MEM_POOL_FRAME_SIZE);
    printf("  Total Allocs:      %lu\r\n", (unsigned long)s_frame_ctrl.stats.total_allocs);
    printf("  Total Frees:       %lu\r\n", (unsigned long)s_frame_ctrl.stats.total_frees);
    printf("  Current Used:      %lu / %d blocks (%lu%%)\r\n", 
           (unsigned long)s_frame_ctrl.stats.current_used, MEM_POOL_FRAME_COUNT,
           (unsigned long)(s_frame_ctrl.stats.current_used * 100) / MEM_POOL_FRAME_COUNT);
    printf("  Peak Used:         %lu blocks\r\n", (unsigned long)s_frame_ctrl.stats.peak_used);
    printf("  Alloc Failures:    %lu\r\n", (unsigned long)s_frame_ctrl.stats.alloc_failures);
    printf("  Invalid Frees:     %lu\r\n", (unsigned long)s_frame_ctrl.stats.invalid_frees);
    printf("  Double Frees:      %lu\r\n", (unsigned long)s_frame_ctrl.stats.double_frees);
    printf("  Leak Warnings:     %lu\r\n", (unsigned long)s_frame_ctrl.stats.leak_warnings);
    
    /* 电机状态池 */
    printf("\r\n[Motor State Pool] (%d x %dB = %dB total)\r\n", 
           MEM_POOL_MOTOR_STATE_COUNT, MEM_POOL_MOTOR_STATE_SIZE,
           MEM_POOL_MOTOR_STATE_COUNT * MEM_POOL_MOTOR_STATE_SIZE);
    printf("  Total Allocs:      %lu\r\n", (unsigned long)s_motor_state_ctrl.stats.total_allocs);
    printf("  Total Frees:       %lu\r\n", (unsigned long)s_motor_state_ctrl.stats.total_frees);
    printf("  Current Used:      %lu / %d blocks (%lu%%)\r\n", 
           (unsigned long)s_motor_state_ctrl.stats.current_used, MEM_POOL_MOTOR_STATE_COUNT,
           (unsigned long)(s_motor_state_ctrl.stats.current_used * 100) / MEM_POOL_MOTOR_STATE_COUNT);
    printf("  Peak Used:         %lu blocks\r\n", (unsigned long)s_motor_state_ctrl.stats.peak_used);
    printf("  Alloc Failures:    %lu\r\n", (unsigned long)s_motor_state_ctrl.stats.alloc_failures);
    printf("  Invalid Frees:     %lu\r\n", (unsigned long)s_motor_state_ctrl.stats.invalid_frees);
    printf("  Double Frees:      %lu\r\n", (unsigned long)s_motor_state_ctrl.stats.double_frees);
    printf("  Leak Warnings:     %lu\r\n", (unsigned long)s_motor_state_ctrl.stats.leak_warnings);
    
    printf("============================================\r\n\r\n");
}

/**
 * @brief  打印详细的泄漏报告
 */
void mem_pool_print_leak_report(uint32_t current_time)
{
    bool leak_found = false;
    
    printf("\r\n========== Memory Leak Report ==========\r\n");
    
    /* 检查帧缓冲池 */
    printf("[Frame Buffer Pool]\r\n");
    for (uint16_t i = 0; i < MEM_POOL_FRAME_COUNT; i++) {
        if (s_frame_meta[i].in_use) {
            uint32_t elapsed = current_time - s_frame_meta[i].alloc_time;
            if (elapsed > MEM_POOL_LEAK_TIMEOUT_MS) {
                printf("  ⚠ Block %d LEAKED: allocated at %s:%d, %.2fs ago\r\n", 
                       i, s_frame_meta[i].alloc_source, s_frame_meta[i].alloc_line,
                       elapsed / 1000.0f);
                leak_found = true;
            }
        }
    }
    if (!leak_found) {
        printf("  ✓ No leaks detected\r\n");
    }
    
    leak_found = false;
    
    /* 检查电机状态池 */
    printf("\r\n[Motor State Pool]\r\n");
    for (uint16_t i = 0; i < MEM_POOL_MOTOR_STATE_COUNT; i++) {
        if (s_motor_state_meta[i].in_use) {
            uint32_t elapsed = current_time - s_motor_state_meta[i].alloc_time;
            if (elapsed > MEM_POOL_LEAK_TIMEOUT_MS) {
                printf("  ⚠ Block %d LEAKED: allocated at %s:%d, %.2fs ago\r\n", 
                       i, s_motor_state_meta[i].alloc_source, s_motor_state_meta[i].alloc_line,
                       elapsed / 1000.0f);
                leak_found = true;
            }
        }
    }
    if (!leak_found) {
        printf("  ✓ No leaks detected\r\n");
    }
    
    printf("========================================\r\n\r\n");
}

/**
 * @brief  重置统计计数器
 */
void mem_pool_reset_stats(mem_pool_type_t type)
{
    mem_pool_ctrl_t *ctrl = (type == MEM_POOL_TYPE_FRAME) ? &s_frame_ctrl : &s_motor_state_ctrl;
    
    /* 保留 current_used，重置其他计数器 */
    uint32_t current = ctrl->stats.current_used;
    memset(&ctrl->stats, 0, sizeof(mem_pool_stats_t));
    ctrl->stats.current_used = current;
}

/* ============================================================================
 * 私有函数实现
 * ============================================================================ */

/**
 * @brief  内部分配函数（核心算法）
 * @param  ctrl: 池控制块指针
 * @param  file: 源文件名
 * @param  line: 源行号
 * @retval 成功返回内存指针，失败返回NULL
 */
static void* mem_pool_alloc_internal(mem_pool_ctrl_t *ctrl, const char *file, uint16_t line)
{
    /* 查找第一个空闲块（O(n)，但n很小，通常≤8） */
    for (uint16_t i = 0; i < ctrl->block_count; i++) {
        if (!ctrl->metadata[i].in_use) {
            /* 标记为已使用 */
            ctrl->metadata[i].in_use = true;
            ctrl->metadata[i].alloc_time = HAL_GetTick();
            ctrl->metadata[i].alloc_source = file;
            ctrl->metadata[i].alloc_line = line;
            
            /* 更新统计 */
            ctrl->stats.total_allocs++;
            ctrl->stats.current_used++;
            if (ctrl->stats.current_used > ctrl->stats.peak_used) {
                ctrl->stats.peak_used = ctrl->stats.current_used;
            }
            
            /* 返回块地址 */
            return ctrl->pool_base + (i * ctrl->block_size);
        }
    }
    
    /* 池已满 */
    ctrl->stats.alloc_failures++;
    return NULL;
}

/**
 * @brief  内部释放函数（核心算法）
 * @param  ctrl: 池控制块指针
 * @param  ptr: 要释放的指针
 * @retval 错误码
 */
static mem_pool_err_t mem_pool_free_internal(mem_pool_ctrl_t *ctrl, void *ptr)
{
    if (ptr == NULL) {
        return MEM_POOL_ERR_INVALID_PTR;
    }
    
    /* 计算块索引 */
    int32_t index = mem_pool_get_block_index(ctrl, ptr);
    if (index < 0) {
        ctrl->stats.invalid_frees++;
        return MEM_POOL_ERR_INVALID_PTR;
    }
    
    /* 检查重复释放 */
    if (!ctrl->metadata[index].in_use) {
        ctrl->stats.double_frees++;
        return MEM_POOL_ERR_DOUBLE_FREE;
    }
    
    /* 标记为空闲 */
    ctrl->metadata[index].in_use = false;
    ctrl->metadata[index].alloc_time = 0;
    ctrl->metadata[index].alloc_source = NULL;
    ctrl->metadata[index].alloc_line = 0;
    
    /* 更新统计 */
    ctrl->stats.total_frees++;
    ctrl->stats.current_used--;
    
    return MEM_POOL_OK;
}

/**
 * @brief  计算指针对应的块索引
 * @param  ctrl: 池控制块指针
 * @param  ptr: 内存指针
 * @retval 成功返回索引[0, block_count-1]，失败返回-1
 */
static int32_t mem_pool_get_block_index(mem_pool_ctrl_t *ctrl, void *ptr)
{
    uint8_t *p = (uint8_t *)ptr;
    
    /* 检查指针是否在池范围内 */
    if (p < ctrl->pool_base || p >= ctrl->pool_base + (ctrl->block_size * ctrl->block_count)) {
        return -1;
    }
    
    /* 计算偏移量 */
    uint32_t offset = p - ctrl->pool_base;
    
    /* 检查是否对齐到块边界 */
    if (offset % ctrl->block_size != 0) {
        return -1;
    }
    
    return (int32_t)(offset / ctrl->block_size);
}
