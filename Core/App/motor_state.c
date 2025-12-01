/**
 ****************************************************************************************************
 * @file        motor_state.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-12-01
 * @brief       电机状态封装模块实现
 ****************************************************************************************************
 */

#include "motor_state.h"
#include <string.h>

/* 私有全局状态（替代usart.c中的分散全局变量） */
static motor_comm_state_t g_motor_comm;
static motor_ctrl_state_t g_motor_ctrl[MAX_MOTOR_COUNT] = {0};  /* 支持多电机 */

/**
 * @brief       初始化电机状态模块
 * @param       无
 * @retval      无
 */
void motor_state_init(void)
{
    memset(&g_motor_comm, 0, sizeof(motor_comm_state_t));
    memset(g_motor_ctrl, 0, sizeof(g_motor_ctrl));
    
    /* 预设默认地址 */
    for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++)
    {
        g_motor_ctrl[i].addr = i + 1;
    }
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

/**
 * @brief       获取电机控制状态
 * @param       addr: 电机地址（1-MAX_MOTOR_COUNT）
 * @retval      控制状态指针（地址非法返回NULL）
 */
motor_ctrl_state_t* motor_get_ctrl_state(uint8_t addr)
{
    if (addr == 0 || addr > MAX_MOTOR_COUNT)
    {
        return NULL;
    }
    return &g_motor_ctrl[addr - 1];
}

/**
 * @brief       更新电机位置 (Phase 3扩展缓存)
 * @param       addr: 电机地址
 * @param       pos: 新位置
 * @retval      无
 */
void motor_update_position(uint8_t addr, int32_t pos)
{
    motor_ctrl_state_t *ctrl = motor_get_ctrl_state(addr);
    if (ctrl != NULL)
    {
        ctrl->current_pos = pos;
        ctrl->last_update_tick = HAL_GetTick();
        ctrl->cache_valid = true;
    }
}

/**
 * @brief       更新电机状态标志 (Phase 3新增)
 * @param       addr: 电机地址
 * @param       flags: 状态标志 (bit0=使能, bit1=到位, bit2=堵转)
 * @retval      无
 */
void motor_update_status(uint8_t addr, uint8_t flags)
{
    motor_ctrl_state_t *ctrl = motor_get_ctrl_state(addr);
    if (ctrl != NULL)
    {
        ctrl->status_flags = flags;
        ctrl->enabled = (flags & 0x01) != 0;
        ctrl->last_update_tick = HAL_GetTick();
        ctrl->cache_valid = true;
    }
}

/**
 * @brief       检查缓存是否有效 (Phase 3新增)
 * @param       addr: 电机地址
 * @retval      true: 缓存有效(<100ms), false: 缓存过期
 */
bool motor_is_cache_valid(uint8_t addr)
{
    motor_ctrl_state_t *ctrl = motor_get_ctrl_state(addr);
    if (ctrl == NULL || !ctrl->cache_valid)
    {
        return false;
    }
    
    /* 检查是否超过100ms */
    uint32_t elapsed = HAL_GetTick() - ctrl->last_update_tick;
    if (elapsed > MOTOR_CACHE_TIMEOUT_MS)
    {
        ctrl->cache_valid = false;
        return false;
    }
    
    return true;
}

/**
 * @brief       失效缓存 (Phase 3新增)
 * @param       addr: 电机地址，0=失效所有电机
 * @retval      无
 */
void motor_invalidate_cache(uint8_t addr)
{
    if (addr == 0)  /* 失效所有缓存 */
    {
        for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++)
        {
            g_motor_ctrl[i].cache_valid = false;
        }
    }
    else  /* 失效指定电机 */
    {
        motor_ctrl_state_t *ctrl = motor_get_ctrl_state(addr);
        if (ctrl != NULL)
        {
            ctrl->cache_valid = false;
        }
    }
}
