/**
 ****************************************************************************************************
 * @file        app_init.h
 * @author      STM32_485 Project Team
 * @version     V3.0
 * @date        2025-12-01
 * @brief       应用层初始化编排（App层）
 ****************************************************************************************************
 * @attention   架构定位：
 *              - App层业务逻辑编排
 *              - 统一初始化流程管理
 *              - 可调用所有下层模块
 ****************************************************************************************************
 */

#ifndef __APP_INIT_H
#define __APP_INIT_H

#include "sys.h"
#include <stdint.h>
#include <stdbool.h>

/* 系统初始化返回码 */
typedef enum {
    BSP_INIT_OK       = 0,
    BSP_INIT_CLOCK_ERROR   = 1,
    BSP_INIT_PERIPH_ERROR  = 2
} bsp_init_status_t;

/* API函数声明 */
bsp_init_status_t bsp_system_init(void);        /* 系统核心初始化（时钟+HAL） */
bsp_init_status_t bsp_peripheral_init(void);    /* 外设初始化（GPIO+USART+定时器） */
void bsp_print_boot_info(void);                 /* 打印启动信息 */

#endif /* __APP_INIT_H */
