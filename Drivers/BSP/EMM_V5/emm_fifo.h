/**
 ****************************************************************************************************
 * @file        emm_fifo.h
 * @author      张大头闭环伺服 + 正点原子团队适配
 * @version     V1.0 (HAL库版本)
 * @date        2025-12-01
 * @brief       FIFO环形缓冲队列（用于电机通信）
 * @license     基于张大头原始代码，适配正点原子M48Z-M3开发板
 ****************************************************************************************************
 * @attention
 * 
 * 原始作者: ZHANGDATOU (https://zhangdatou.taobao.com)
 * HAL库适配: 正点原子团队(ALIENTEK)
 * 实验平台: 正点原子 M48Z-M3最小系统板STM32F103版
 * 
 * 功能说明:
 * - 环形队列，用于串口接收数据缓冲
 * - 支持中断中快速入队，主循环中批量出队
 * - 队列大小128字节
 * 
 ****************************************************************************************************
 */

#ifndef __EMM_FIFO_H
#define __EMM_FIFO_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 宏定义 (Phase 2扩容优化) */
#define EMM_FIFO_SIZE   512                         /* FIFO深度(原128→512) */
#define EMM_FIFO_WARN_LEVEL  410                    /* 水位警告阈值(80%) */

/* FIFO结构体定义 (Phase 2修正: 指针类型匹配512深度) */
typedef struct {
    uint16_t buffer[EMM_FIFO_SIZE];                 /* 数据缓冲区 */
    __IO uint16_t ptrWrite;                         /* 写指针(uint16_t支持512) */
    __IO uint16_t ptrRead;                          /* 读指针(uint16_t支持512) */
} EMM_FIFO_t;

/* 外部变量声明 */
extern __IO EMM_FIFO_t g_emm_rx_fifo;               /* 接收FIFO队列 */

/* 函数声明 */
void emm_fifo_init(void);                           /* 初始化队列 */
uint8_t emm_fifo_enqueue(uint16_t data);            /* 入队（中断中调用），返回0=成功,1=队列满 */
uint16_t emm_fifo_dequeue(void);                    /* 出队 */
uint8_t emm_fifo_is_empty(void);                    /* 判断队列是否为空 */
uint16_t emm_fifo_length(void);                     /* 计算队列长度 */
uint8_t emm_fifo_get_usage_percent(void);           /* 获取FIFO使用率(0-100) - Phase 2新增 */
uint8_t emm_fifo_is_high_water(void);               /* 检查是否超过80%水位 - Phase 2新增 */

#endif
