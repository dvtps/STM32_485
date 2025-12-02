/**
 ****************************************************************************************************
 * @file        fifo.h
 * @author      STM32_485 Project (SYSTEM Layer)
 * @version     V3.0
 * @date        2025-12-01
 * @brief       通用FIFO环形缓冲队列（SYSTEM层基础设施）
 ****************************************************************************************************
 * @attention
 * 
 * 架构定位: SYSTEM层 - 通用数据结构
 * 实验平台: 正点原子 M48Z-M3最小系统板STM32F103版
 * 
 * 功能说明:
 * - 通用环形队列实现
 * - 支持中断中快速入队，主循环中批量出队
 * - 线程安全（通过关中断保护）
 * - 队列大小256字节
 * 
 ****************************************************************************************************
 */

#ifndef __FIFO_H
#define __FIFO_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 宏定义 (V3.5 Phase 8优化: 扩大FIFO容量，提升稳定性) */
#define EMM_FIFO_SIZE   256                         /* FIFO深度: 256字节(电机帧最大20B，可缓存12帧，Modbus+电机并发安全) */
#define EMM_FIFO_WARN_LEVEL  205                    /* 水位警告阈值(80%) */

/* FIFO结构体定义 (V3.5 Phase 8: 指针类型升级为uint16_t，支持256字节容量) */
typedef struct {
    uint16_t buffer[EMM_FIFO_SIZE];                 /* 数据缓冲区 (uint16_t保存特殊状态) */
    __IO uint16_t ptrWrite;                         /* 写指针(uint16_t支持256) */
    __IO uint16_t ptrRead;                          /* 读指针(uint16_t支持256) */
} EMM_FIFO_t;

/* FIFO统计信息结构体 (V3.5生产级优化新增) */
typedef struct {
    uint32_t enqueue_ok_cnt;                        /* 成功入队计数 */
    uint32_t enqueue_overflow_cnt;                  /* 溢出丢弃计数 */
    uint32_t dequeue_cnt;                           /* 出队计数 */
    uint32_t high_water_mark;                       /* 历史最高水位 */
    uint32_t overflow_last_tick;                    /* 最后一次溢出时间戳 */
} EMM_FIFO_stats_t;

/* 外部变量声明 */
extern __IO EMM_FIFO_t g_emm_rx_fifo;               /* 接收FIFO队列 */
extern EMM_FIFO_stats_t g_emm_fifo_stats;           /* FIFO统计信息 */

/* 函数声明 */
void emm_fifo_init(void);                           /* 初始化队列 */
uint8_t emm_fifo_enqueue(uint16_t data);            /* 入队（中断中调用），返回0=成功,1=队列满 */
uint16_t emm_fifo_dequeue(void);                    /* 出队 */
uint8_t emm_fifo_is_empty(void);                    /* 判断队列是否为空 */
uint16_t emm_fifo_length(void);                     /* 计算队列长度 */
uint8_t emm_fifo_get_usage_percent(void);           /* 获取FIFO使用率(0-100) - Phase 2新增 */
uint8_t emm_fifo_is_high_water(void);               /* 检查是否超过80%水位 - Phase 2新增 */
void emm_fifo_get_stats(EMM_FIFO_stats_t *stats);   /* 获取统计信息 - V3.5新增 */
void emm_fifo_clear_stats(void);                    /* 清除统计信息 - V3.5新增 */

#endif /* __FIFO_H */
