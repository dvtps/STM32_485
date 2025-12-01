/**
 ****************************************************************************************************
 * @file        sys_timer.h
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-02
 * @brief       非阻塞软件定时器（替代HAL_Delay）
 ****************************************************************************************************
 * @attention
 * 
 * 功能特性:
 * - 非阻塞定时
 * - 支持多个独立定时器
 * - 自动重载/单次触发模式
 * - 基于SysTick，1ms精度
 * 
 * 使用示例:
 * @code
 * sys_timer_t led_timer;
 * sys_timer_start(&led_timer, 500, false);  // 500ms单次定时
 * 
 * // 主循环中
 * if (sys_timer_expired(&led_timer)) {
 *     toggle_led();
 *     sys_timer_start(&led_timer, 500, false);  // 重新启动
 * }
 * @endcode
 * 
 ****************************************************************************************************
 */

#ifndef __SYS_TIMER_H
#define __SYS_TIMER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================== 定时器结构体 ======================== */

/**
 * @brief 软件定时器结构体
 */
typedef struct {
    uint32_t start_tick;            /* 启动时间戳 */
    uint32_t timeout_ms;            /* 超时时间(ms) */
    bool active;                    /* 是否激活 */
    bool auto_reload;               /* 自动重载标志 */
} sys_timer_t;

/* ======================== 函数声明 ======================== */

void sys_timer_start(sys_timer_t *timer, uint32_t timeout_ms, bool auto_reload);
void sys_timer_stop(sys_timer_t *timer);
bool sys_timer_expired(sys_timer_t *timer);
uint32_t sys_timer_remaining(const sys_timer_t *timer);
bool sys_timer_is_active(const sys_timer_t *timer);

#endif /* __SYS_TIMER_H */
