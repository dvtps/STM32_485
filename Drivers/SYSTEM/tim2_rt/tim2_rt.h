/**
 ****************************************************************************************************
 * @file        tim2_rt.h
 * @author      STM32_485 Project
 * @version     V1.0
 * @date        2025-12-03
 * @brief       TIM2定时器配置头文件（实时电机10kHz控制周期）
 ****************************************************************************************************
 */

#ifndef __TIM2_RT_H
#define __TIM2_RT_H

#include "sys.h"
#include "stm32f1xx_hal.h"

/* TIM2句柄声明 */
extern TIM_HandleTypeDef g_htim2_rt;

/* 函数声明 */
uint8_t tim2_rt_init(void);       /* TIM2初始化 */
void tim2_rt_start(void);         /* 启动定时器 */
void tim2_rt_stop(void);          /* 停止定时器 */
uint8_t tim2_rt_is_running(void); /* 获取运行状态 */

#endif /* __TIM2_RT_H */
