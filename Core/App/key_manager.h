/**
 ****************************************************************************************************
 * @file        key_manager.h
 * @author      STM32_485 Project
 * @version     V3.2
 * @date        2025-12-02
 * @brief       按键管理器（App层：去抖+状态机）
 ****************************************************************************************************
 * @attention
 *
 * 架构定位：App层 - 业务逻辑
 * - 从BSP层读取原始GPIO电平（key_read_key0/wkup）
 * - 实现去抖状态机（IDLE→DEBOUNCE→PRESSED）
 * - 支持连按模式
 * - 提供与旧版key_scan()兼容的接口
 *
 ****************************************************************************************************
 */

#ifndef __KEY_MANAGER_H
#define __KEY_MANAGER_H

#include <stdint.h>

/* 按键状态枚举（App层业务逻辑） */
typedef enum {
    KEY_MGR_STATE_IDLE = 0,        /* 空闲状态 */
    KEY_MGR_STATE_DEBOUNCE,        /* 去抖动状态 */
    KEY_MGR_STATE_PRESSED          /* 按下确认状态 */
} key_manager_state_t;

/* 按键值定义（与BSP层保持一致） */
#define KEY_MGR_NO_PRESS    0
#define KEY_MGR_KEY0_PRESS  1
#define KEY_MGR_WKUP_PRESS  2

/* 配置参数 */
#define KEY_MGR_DEBOUNCE_MS    10    /* 去抖时间10ms */

/******************************************************************************************/
/* API函数 */

void key_manager_init(void);                 /* 初始化按键管理器 */
uint8_t key_manager_scan(uint8_t mode);      /* 扫描按键（兼容旧版key_scan） */
void key_manager_reset(void);                /* 重置状态机（调试用） */

#endif

