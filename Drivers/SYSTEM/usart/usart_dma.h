/**
 ****************************************************************************************************
 * @file        usart_dma.h
 * @author      正点原子团队(ALIENTEK) - Phase 3高级优化
 * @version     V1.0
 * @date        2025-12-01
 * @brief       USART2 DMA接收头文件
 ****************************************************************************************************
 */

#ifndef __USART_DMA_H
#define __USART_DMA_H

#include "stm32f1xx_hal.h"

/* 函数声明 */
void usart2_dma_init(void);                         /* 初始化USART2 DMA接收 */
void usart2_idle_dma_handler(UART_HandleTypeDef *huart);  /* IDLE中断处理（DMA版本） */
uint8_t usart2_dma_get_usage(void);                 /* 获取DMA缓冲区使用率 */

#endif
