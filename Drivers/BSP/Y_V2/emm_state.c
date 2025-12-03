/**
 ****************************************************************************************************
 * @file        emm_state.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V3.2 (架构重构版)
 * @date        2025-12-02
 * @brief       电机通信状态封装（BSP层：仅硬件相关）
 ****************************************************************************************************
 * @attention
 *
 * 架构定位：BSP层 - 硬件抽象（无业务状态）
 * - V3.2移除电机控制状态（g_motor_ctrl） → 迁移到MULTI_MOTOR中间件
 * - BSP层仅保留通信状态（接收缓冲区、帧标志）
 * - 电机位置/速度/缓存管理由上层负责
 *
 ****************************************************************************************************
 */

#include "emm_state.h"
#include <string.h>

/* 私有全局状态（BSP层：仅通信相关） */
static motor_comm_state_t g_motor_comm;

/**
 * @brief       初始化电机通信状态
 * @param       无
 * @retval      无
 */
void motor_state_init(void)
{
    memset(&g_motor_comm, 0, sizeof(motor_comm_state_t));
}

/**
 * @brief       获取通信状态结构体指针
 * @param       无
 * @retval      通信状态指针
 */
motor_comm_state_t* motor_get_comm_state(void)
{
    return &g_motor_comm;
}

/**
 * @brief       获取命令帧缓冲区
 * @param       无
 * @retval      命令帧指针（兼容旧代码g_emm_rx_cmd访问方式）
 */
uint8_t* motor_get_cmd_frame(void)
{
    return g_motor_comm.cmd_frame;
}

/**
 * @brief       获取当前帧长度
 * @param       无
 * @retval      帧长度（兼容旧代码g_emm_rx_count）
 */
uint16_t motor_get_frame_len(void)
{
    return g_motor_comm.frame_len;
}

/**
 * @brief       检查帧是否完成
 * @param       无
 * @retval      true: 帧完成, false: 未完成
 */
bool motor_is_frame_complete(void)
{
    return g_motor_comm.frame_complete != 0;
}

/**
 * @brief       清除帧完成标志
 * @param       无
 * @retval      无
 */
void motor_clear_frame_flag(void)
{
    g_motor_comm.frame_complete = 0;
}

/* ========== V3.2架构重构：以下控制状态函数已移除 ========== */
/* 电机位置/速度/缓存管理请使用MULTI_MOTOR中间件的API */
/* - multi_motor_get_position() */
/* - multi_motor_update_status() */
/* - motor_info_t结构体 */
