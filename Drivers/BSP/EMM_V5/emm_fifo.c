/**
 ****************************************************************************************************
 * @file        emm_fifo.c
 * @author      张大头闭环伺服 + 正点原子团队适配
 * @version     V1.0 (HAL库版本)
 * @date        2025-12-01
 * @brief       FIFO环形缓冲队列实现
 * @license     基于张大头原始代码，适配正点原子M48Z-M3开发板
 ****************************************************************************************************
 * @attention
 * 
 * 原始作者: ZHANGDATOU (https://zhangdatou.taobao.com)
 * HAL库适配: 正点原子团队(ALIENTEK)
 * 
 * 修改说明:
 * - 标准库 → HAL库
 * - 函数名统一加emm_fifo前缀
 * - 全局变量加g_前缀
 * 
 ****************************************************************************************************
 */

#include "emm_fifo.h"

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
