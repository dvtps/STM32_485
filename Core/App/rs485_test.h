/**
 ****************************************************************************************************
 * @file        rs485_test.h
 * @author      STM32_485项目
 * @version     V1.0
 * @date        2025-12-01
 * @brief       RS485通信测试模块头文件
 ****************************************************************************************************
 */

#ifndef __RS485_TEST_H
#define __RS485_TEST_H

#include "stm32f1xx_hal.h"

/* 函数声明 */
void rs485_test_init(void);             /* 测试初始化 */
void rs485_test_run(void);              /* 测试主循环 */
void rs485_send_byte(uint8_t data);     /* 发送单字节（USMART） */
void rs485_send_string(char *str);      /* 发送字符串（USMART） */
void rs485_test_gpio(void);             /* GPIO状态检查 */
void rs485_print_config(void);          /* 打印USART2配置 */

#endif /* __RS485_TEST_H */
