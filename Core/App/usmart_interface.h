/**
 ****************************************************************************************************
 * @file        usmart_interface.h
 * @author      Project Team
 * @version     V1.0
 * @date        2025-12-01
 * @brief       USMART 串口调试接口封装
 * @note        此文件提供电机控制等功能的简化接口，供USMART串口调试工具调用
 ****************************************************************************************************
 */

#ifndef __USMART_INTERFACE_H
#define __USMART_INTERFACE_H

#include "main.h"

/******************************************************************************************/
/* BSP/LED 模块 */
#include "led.h"

/* BSP/KEY 模块 */
#include "key.h"

/* BSP/IWDG 模块 */
#include "iwdg.h"

/* BSP/EMM_V5 电机控制模块 */
#include "emm_v5.h"

/******************************************************************************************/
/* 电机控制封装函数（简化USMART调用接口） */

/**
 * @brief       电机使能控制
 * @param       addr: 电机地址 (1-255)
 * @param       enable: true=使能, false=失能
 * @retval      无
 */
static inline void motor_enable(uint8_t addr, uint8_t enable)
{
    Emm_V5_En_Control(addr, enable ? true : false, false);
}

/**
 * @brief       电机位置模式运动（相对运动）
 * @param       addr: 电机地址
 * @param       dir: 方向 0=CW, 1=CCW
 * @param       speed: 速度 (0-5000 RPM)
 * @param       acc: 加速度 (0-255)
 * @param       pulses: 脉冲数 (16细分: 3200=1圈, 1600=半圈)
 * @retval      无
 */
static inline void motor_pos_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    Emm_V5_Pos_Control(addr, dir, speed, acc, pulses, false, false);
}

/**
 * @brief       电机速度模式运动
 * @param       addr: 电机地址
 * @param       dir: 方向 0=CW, 1=CCW
 * @param       speed: 速度 (0-5000 RPM)
 * @param       acc: 加速度 (0-255)
 * @retval      无
 */
static inline void motor_vel_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc)
{
    Emm_V5_Vel_Control(addr, dir, speed, acc, false);
}

/**
 * @brief       电机急停
 * @param       addr: 电机地址
 * @retval      无
 */
static inline void motor_stop(uint8_t addr)
{
    Emm_V5_Stop_Now(addr, false);
}

/**
 * @brief       电机回零（模式0：单圈就近回零）
 * @param       addr: 电机地址
 * @retval      无
 */
static inline void motor_home(uint8_t addr)
{
    Emm_V5_Origin_Trigger_Return(addr, 0, false);
}

/**
 * @brief       读取电机状态
 * @param       addr: 电机地址
 * @retval      无
 */
static inline void motor_read_status(uint8_t addr)
{
    Emm_V5_Read_Sys_Params(addr, S_VER); // 读取固件版本号
}

#endif /* __USMART_INTERFACE_H */
