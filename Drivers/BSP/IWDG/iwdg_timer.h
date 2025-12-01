/**
 ****************************************************************************************************
 * @file        iwdg_timer.h
 * @author      正点原子团队(ALIENTEK) - Phase 4高级优化
 * @version     V1.0
 * @date        2025-12-01
 * @brief       基于TIM3的定时看门狗喂养头文件
 ****************************************************************************************************
 */

#ifndef __IWDG_TIMER_H
#define __IWDG_TIMER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* 函数声明 */
void tim3_watchdog_init(void);                  /* 初始化TIM3定时看门狗 */
void main_loop_heartbeat_update(void);          /* 主循环心跳更新 */
uint32_t get_health_check_fail_count(void);     /* 获取健康检查失败次数 */

#endif
