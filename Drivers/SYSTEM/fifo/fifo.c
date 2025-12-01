/**
 ****************************************************************************************************
 * @file        fifo.c
 * @author      STM32_485 Project (SYSTEM Layer)
 * @version     V3.0
 * @date        2025-12-01
 * @brief       通用FIFO环形缓冲队列实现（SYSTEM层）
 ****************************************************************************************************
 * @attention
 * 
 * 架构定位: SYSTEM层 - 通用数据结构，不依赖任何上层
 * 
 * 版本历史:
 * V1.0 - 基于张大头代码适配HAL库
 * V3.0 - 重构为SYSTEM层通用模块
 * 
 ****************************************************************************************************
 */

#include "fifo.h"

/* 全局变量定义 */
__IO EMM_FIFO_t g_emm_rx_fifo = {0};
EMM_FIFO_stats_t g_emm_fifo_stats = {0};            /* FIFO统计信息 */

/**
 * @brief       初始化FIFO队列
 * @param       无
 * @retval      无
 */
void emm_fifo_init(void)
{
    g_emm_rx_fifo.ptrRead  = 0;
    g_emm_rx_fifo.ptrWrite = 0;
    
    /* 初始化统计信息 */
    g_emm_fifo_stats.enqueue_ok_cnt = 0;
    g_emm_fifo_stats.enqueue_overflow_cnt = 0;
    g_emm_fifo_stats.dequeue_cnt = 0;
    g_emm_fifo_stats.high_water_mark = 0;
    g_emm_fifo_stats.overflow_last_tick = 0;
}

/**
 * @brief       数据入队（在中断中调用）
 * @param       data: 要入队的数据
 * @retval      0: 成功, 1: 队列已满
 * @note        V3.5增强: 添加溢出统计和水位监控
 */
uint8_t emm_fifo_enqueue(uint16_t data)
{
    uint16_t next_write = g_emm_rx_fifo.ptrWrite + 1;
    uint16_t current_len;
    
    /* 环绕处理 */
    if (next_write >= EMM_FIFO_SIZE)
    {
        next_write = 0;
    }
    
    /* 检查队列满（写指针追上读指针） */
    if (next_write == g_emm_rx_fifo.ptrRead)
    {
        /* 溢出统计 */
        g_emm_fifo_stats.enqueue_overflow_cnt++;
        g_emm_fifo_stats.overflow_last_tick = HAL_GetTick();
        return 1;  /* 队列已满，丢弃数据 */
    }
    
    /* 写入数据 */
    g_emm_rx_fifo.buffer[g_emm_rx_fifo.ptrWrite] = data;
    g_emm_rx_fifo.ptrWrite = next_write;
    
    /* 成功统计 */
    g_emm_fifo_stats.enqueue_ok_cnt++;
    
    /* 更新历史最高水位 */
    current_len = emm_fifo_length();
    if (current_len > g_emm_fifo_stats.high_water_mark)
    {
        g_emm_fifo_stats.high_water_mark = current_len;
    }
    
    return 0;  /* 成功 */
}

/**
 * @brief       数据出队
 * @param       无
 * @retval      出队的数据
 * @note        V3.5增强: 添加出队统计
 */
uint16_t emm_fifo_dequeue(void)
{
    uint16_t element = 0;

    element = g_emm_rx_fifo.buffer[g_emm_rx_fifo.ptrRead];

    ++g_emm_rx_fifo.ptrRead;

    if (g_emm_rx_fifo.ptrRead >= EMM_FIFO_SIZE)
    {
        g_emm_rx_fifo.ptrRead = 0;
    }
    
    /* 出队统计 */
    g_emm_fifo_stats.dequeue_cnt++;

    return element;
}

/**
 * @brief       判断队列是否为空
 * @param       无
 * @retval      1: 队列为空, 0: 队列非空
 */
uint8_t emm_fifo_is_empty(void)
{
    if (g_emm_rx_fifo.ptrRead == g_emm_rx_fifo.ptrWrite)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief       计算队列当前长度
 * @param       无
 * @retval      队列中的数据个数
 */
uint16_t emm_fifo_length(void)
{
    if (g_emm_rx_fifo.ptrRead <= g_emm_rx_fifo.ptrWrite)
    {
        return (g_emm_rx_fifo.ptrWrite - g_emm_rx_fifo.ptrRead);
    }
    else
    {
        return (EMM_FIFO_SIZE - g_emm_rx_fifo.ptrRead + g_emm_rx_fifo.ptrWrite);
    }
}

/**
 * @brief       获取FIFO使用率 (Phase 2新增)
 * @param       无
 * @retval      使用率百分比(0-100)
 */
uint8_t emm_fifo_get_usage_percent(void)
{
    uint16_t len = emm_fifo_length();
    return (uint8_t)((len * 100) / EMM_FIFO_SIZE);
}

/**
 * @brief       检查FIFO是否超过高水位 (Phase 2新增)
 * @param       无
 * @retval      1: 超过80%水位, 0: 正常
 */
uint8_t emm_fifo_is_high_water(void)
{
    uint16_t len = emm_fifo_length();
    return (len >= EMM_FIFO_WARN_LEVEL) ? 1 : 0;
}

/**
 * @brief       获取FIFO统计信息 (V3.5新增)
 * @param       stats: 输出统计数据的指针
 * @retval      无
 * @note        原子操作，可在中断中调用
 */
void emm_fifo_get_stats(EMM_FIFO_stats_t *stats)
{
    if (stats == NULL) return;
    
    __disable_irq();
    stats->enqueue_ok_cnt = g_emm_fifo_stats.enqueue_ok_cnt;
    stats->enqueue_overflow_cnt = g_emm_fifo_stats.enqueue_overflow_cnt;
    stats->dequeue_cnt = g_emm_fifo_stats.dequeue_cnt;
    stats->high_water_mark = g_emm_fifo_stats.high_water_mark;
    stats->overflow_last_tick = g_emm_fifo_stats.overflow_last_tick;
    __enable_irq();
}

/**
 * @brief       清除FIFO统计信息 (V3.5新增)
 * @param       无
 * @retval      无
 */
void emm_fifo_clear_stats(void)
{
    __disable_irq();
    g_emm_fifo_stats.enqueue_ok_cnt = 0;
    g_emm_fifo_stats.enqueue_overflow_cnt = 0;
    g_emm_fifo_stats.dequeue_cnt = 0;
    g_emm_fifo_stats.high_water_mark = 0;
    g_emm_fifo_stats.overflow_last_tick = 0;
    __enable_irq();
}
