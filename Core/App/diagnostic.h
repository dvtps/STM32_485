/**
 ****************************************************************************************************
 * @file        diagnostic.h
 * @author      STM32_485 Project Team
 * @version     V3.0
 * @date        2025-12-01
 * @brief       系统诊断模块
 ****************************************************************************************************
 * @attention   工业级功能：
 *              - 时钟频率检测和验证
 *              - USART配置诊断
 *              - LED闪烁显示系统状态
 ****************************************************************************************************
 */

#ifndef __DIAGNOSTIC_H
#define __DIAGNOSTIC_H

#include "sys.h"
#include <stdint.h>
#include <stdbool.h>

/* API函数声明 */
void diag_run_full_check(void);                  /* 运行完整诊断流程 */
void diag_print_clock_info(void);                /* 打印时钟信息 */
void diag_print_usart_config(void);              /* 打印USART配置 */
void diag_blink_clock_speed(void);               /* 用LED闪烁显示时钟频率 */

#endif
