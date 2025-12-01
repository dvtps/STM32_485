/**
 ****************************************************************************************************
 * @file        atk_rs485.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_RS485模块 驱动代码
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
 ****************************************************************************************************
 */

#ifndef __ATK_RS485_H
#define __ATK_RS485_H

#include "./SYSTEM/sys/sys.h"


/**************************************************************************************************/
/* 引脚 定义 */

#define ATK_RS485_TX_GPIO_PORT                  GPIOA
#define ATK_RS485_TX_GPIO_PIN                   GPIO_PIN_2
#define ATK_RS485_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define ATK_RS485_RX_GPIO_PORT                  GPIOA
#define ATK_RS485_RX_GPIO_PIN                   GPIO_PIN_3
#define ATK_RS485_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define ATK_RS485_UX                            USART2
#define ATK_RS485_UX_IRQn                       USART2_IRQn
#define ATK_RS485_UX_IRQHandler                 USART2_IRQHandler
#define ATK_RS485_UX_CLK_ENABLE()               do{ __HAL_RCC_USART2_CLK_ENABLE(); }while(0)  /* USART2 时钟使能 */

#define RS485_REC_LEN       220                 /* 定义最大接收字节数 64 */
#define RS485_EN_RX         1                   /* 使能（1）/禁止（0）RS485接收 */

extern uint8_t g_RS485_rx_buf[RS485_REC_LEN];   /* 接收缓冲,最大RS485_REC_LEN个字节 */
extern uint8_t g_RS485_rx_cnt;                  /* 接收数据长度 */

/******************************************************************************************/

void atk_rs485_init( uint32_t baudrate);                 /* RS485初始化 */
void atk_rs485_send_data(uint8_t *buf, uint8_t len);     /* RS485发送数据 */
void atk_rs485_receive_data(uint8_t *buf, uint8_t *len); /* RS485接收数据 */

#endif





