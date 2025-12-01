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


/******************************************************************************************/
/* USART1配置 - 调试串口(printf输出) */

#define USART1_TX_GPIO_PORT                 GPIOA
#define USART1_TX_GPIO_PIN                  GPIO_PIN_9
#define USART1_RX_GPIO_PORT                 GPIOA
#define USART1_RX_GPIO_PIN                  GPIO_PIN_10

#define USART1_REC_LEN                      256         /* USART1最大接收字节数(Modbus帧) */
#define USART1_EN_RX                        1           /* 使能（1）/禁止（0）串口1接收 */

/* USART1工作模式选择（工业级设计：运行时可切换） */
typedef enum {
    USART1_MODE_DEBUG = 0,      /* 调试模式: printf输出 + USMART命令接收 */
    USART1_MODE_MODBUS = 1,     /* Modbus模式: Modbus RTU协议通讯 */
} usart1_mode_t;

/******************************************************************************************/
/* USART2配置 - RS485电机通信串口 */

#define USART2_TX_GPIO_PORT                 GPIOA
#define USART2_TX_GPIO_PIN                  GPIO_PIN_2
#define USART2_RX_GPIO_PORT                 GPIOA
#define USART2_RX_GPIO_PIN                  GPIO_PIN_3

#define USART2_REC_LEN                      220         /* USART2最大接收字节数(电机帧) */
#define USART2_EN_RX                        1           /* 使能（1）/禁止（0）串口2接收 */

/* IDLE中断标志清除宏（用于电机帧检测） */
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

/* USART1接收相关(调试串口) */
extern uint8_t  g_usart1_rx_buf[USART1_REC_LEN];        /* USART1接收缓冲 */
extern uint16_t g_usart1_rx_sta;                        /* USART1接收状态标记 */

/* USART1 Modbus模式相关 */
extern volatile usart1_mode_t g_usart1_mode;            /* USART1工作模式 */
extern uint8_t  g_usart1_modbus_rx_buf[USART1_REC_LEN]; /* Modbus接收缓冲 */
extern uint16_t g_usart1_modbus_rx_index;               /* Modbus接收索引 */
extern volatile uint8_t g_usart1_modbus_frame_complete; /* Modbus帧完成标志 */

/* USMART兼容定义(指向USART1接收缓冲区) */
#define g_usart_rx_buf  g_usart1_rx_buf
#define g_usart_rx_sta  g_usart1_rx_sta

/* USART2接收相关(电机串口) */
extern uint8_t g_usart2_rx_buf[USART2_REC_LEN];         /* USART2接收缓冲 */
extern uint8_t g_emm_rx_cmd[USART2_REC_LEN];            /* 电机命令帧缓冲 */
extern uint16_t g_emm_rx_count;                         /* 电机帧长度 */
extern volatile uint8_t g_emm_frame_complete;           /* 电机帧完成标志 */


/* 函数声明 */
void usart1_init(uint32_t baudrate);                    /* USART1初始化(printf调试) */
void usart2_init(uint32_t baudrate);                    /* USART2初始化(RS485电机) */
void usart_init(uint32_t baudrate);                     /* 统一初始化接口(兼容旧代码) */

/* USART1模式控制函数(工业级双模切换) */
void usart1_set_mode(usart1_mode_t mode);               /* 设置USART1工作模式 */
usart1_mode_t usart1_get_mode(void);                    /* 获取当前工作模式 */
void usart1_flush_rx_buffer(void);                      /* 清空接收缓冲区 */

#endif
