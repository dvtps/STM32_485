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

/**
 * @brief       初始化FIFO队列
 * @param       无
 * @retval      无
 */
void emm_fifo_init(void)
{
    g_emm_rx_fifo.ptrRead  = 0;
    g_emm_rx_fifo.ptrWrite = 0;
}

/**
 * @brief       数据入队（在中断中调用）
 * @param       data: 要入队的数据
 * @retval      0: 成功, 1: 队列已满
 */
uint8_t emm_fifo_enqueue(uint16_t data)
{
    uint16_t next_write = g_emm_rx_fifo.ptrWrite + 1;
    
    /* 环绕处理 */
    if (next_write >= EMM_FIFO_SIZE)
    {
        next_write = 0;
    }
    
    /* 检查队列满（写指针追上读指针） */
    if (next_write == g_emm_rx_fifo.ptrRead)
    {
        return 1;  /* 队列已满，丢弃数据 */
    }
    
    /* 写入数据 */
    g_emm_rx_fifo.buffer[g_emm_rx_fifo.ptrWrite] = data;
    g_emm_rx_fifo.ptrWrite = next_write;
    
    return 0;  /* 成功 */
}

/**
 * @brief       数据出队
 * @param       无
 * @retval      出队的数据
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
