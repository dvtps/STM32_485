/**
 ****************************************************************************************************
 * @file        usart1_rs485_test.h
 * @author      STM32_485项目
 * @version     V1.0
 * @date        2025-12-01
 * @brief       USART1 RS485测试模块头文件
 ****************************************************************************************************
 */

#ifndef __USART1_RS485_TEST_H
#define __USART1_RS485_TEST_H

#include "stm32f1xx_hal.h"

/* 函数声明 */
void usart1_rs485_test_init(void);              /* 测试初始化 */
void usart1_rs485_test_run(void);               /* 测试主循环 */
void usart1_rs485_send_byte(uint8_t data);      /* 发送单字节 */
void usart1_rs485_send_string(const char *str); /* 发送字符串 */
void usart1_rs485_check_config(void);           /* 检查配置 */

#endif /* __USART1_RS485_TEST_H */
