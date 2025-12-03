/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      正点原子团队(ALIENTEK) + ZDT项目优化
 * @version     V2.0 (整合USART1+USART2)
 * @date        2025-12-01
 * @brief       串口统一管理模块(USART1:printf调试 + USART2:RS485电机)
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M48Z-M3最小系统板STM32F103版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 功能说明:
 * - USART1 (PA9/PA10): 用于printf调试输出,支持标准输入输出
 * - USART2 (PA2/PA3):  用于RS485电机通信,支持IDLE中断+FIFO
 *
 ****************************************************************************************************
 */

#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"
#include "app_config.h"  /* 实时模式配置 */


/******************************************************************************************/
/* USART1配置 - 调试串口(printf输出) */

#define USART1_TX_GPIO_PORT                 GPIOA
#define USART1_TX_GPIO_PIN                  GPIO_PIN_9
#define USART1_RX_GPIO_PORT                 GPIOA
#define USART1_RX_GPIO_PIN                  GPIO_PIN_10

#define USART1_REC_LEN                      256         /* USART1最大接收字节数 */
#define USART1_EN_RX                        1           /* 使能串口1接收 */

/******************************************************************************************/
/* USART2配置 - RS485电机通信串口 */

#define USART2_TX_GPIO_PORT                 GPIOA
#define USART2_TX_GPIO_PIN                  GPIO_PIN_2
#define USART2_RX_GPIO_PORT                 GPIOA
#define USART2_RX_GPIO_PIN                  GPIO_PIN_3

#define USART2_REC_LEN                      220         /* USART2最大接收字节数 */
#define USART2_EN_RX                        1           /* 使能串口2接收 */

/* IDLE中断标志清除宏 */
#ifndef __HAL_UART_CLEAR_IDLEFLAG
#define __HAL_UART_CLEAR_IDLEFLAG(__HANDLE__)   \
    do{                                          \
        __IO uint32_t tmpreg = 0x00U;            \
        tmpreg = (__HANDLE__)->Instance->SR;     \
        tmpreg = (__HANDLE__)->Instance->DR;     \
        UNUSED(tmpreg);                          \
    } while(0U)
#endif

/******************************************************************************************/

/* 全局变量声明 */
extern UART_HandleTypeDef g_uart1_handle;               /* USART1句柄 */
extern UART_HandleTypeDef g_uart2_handle;               /* USART2句柄 */

#if REALTIME_MOTOR_ENABLE
extern DMA_HandleTypeDef hdma_usart2_tx;                /* USART2 DMA发送句柄 */
#endif

/* USART1接收相关 */
extern uint8_t  g_usart1_rx_buf[USART1_REC_LEN];
extern uint16_t g_usart1_rx_sta;

/* USART2接收相关（电机通信） */
#if USART2_EN_RX
extern uint8_t g_usart2_rx_buf[USART2_REC_LEN];
extern uint8_t g_emm_rx_cmd[USART2_REC_LEN];
extern uint16_t g_emm_rx_count;
extern volatile uint8_t g_emm_frame_complete;

/* V3.6 DMA接收监控函数 */
uint8_t usart2_dma_get_usage(void);                     /* 获取DMA缓冲区使用率 */
void usart2_dma_reset_stats(void);                      /* 重置DMA统计信息 */
#endif

/* USMART兼容定义 */
#define g_usart_rx_buf  g_usart1_rx_buf
#define g_usart_rx_sta  g_usart1_rx_sta

/* USART2接收相关(电机串口) */
extern uint8_t g_usart2_rx_buf[USART2_REC_LEN];         /* USART2接收缓冲 */
extern uint8_t g_emm_rx_cmd[USART2_REC_LEN];            /* 电机命令帧缓冲 */
extern uint16_t g_emm_rx_count;                         /* 电机帧长度 */
extern volatile uint8_t g_emm_frame_complete;           /* 电机帧完成标志 */

/* V3.5 Phase 2: USART2帧就绪标志（主循环轮询） */
extern volatile uint8_t g_usart2_frame_ready;


/* 函数声明 */
void usart1_init(uint32_t baudrate);                    /* USART1初始化(printf调试) */
void usart2_init(uint32_t baudrate);                    /* USART2初始化(RS485电机) */
void usart_init(uint32_t baudrate);                     /* 统一初始化接口(兼容旧代码) */

/* USART2统计与调试函数 */
uint32_t get_idle_interrupt_count(void);                /* 获取IDLE中断计数 */
uint32_t get_fifo_overflow_count(void);                 /* 获取FIFO溢出计数 */

/* V3.5 Phase 8 P1优化: 增量CRC校验函数 */
void reset_incremental_crc(void);                       /* 重置增量CRC状态 */
uint16_t get_incremental_crc(void);                     /* 获取当前累积的CRC值 */
uint16_t get_crc_byte_count(void);                      /* 获取已计算的字节数 */
uint32_t get_crc_calc_count(void);                      /* 获取CRC计算帧数统计 */

#endif
