/**
 ******************************************************************************
 * @file    modbus_adapter.c
 * @author  STM32_485 Project (App Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   Modbus回调适配器 - 连接Middlewares层与BSP层
 ******************************************************************************
 * @attention
 * 
 * 架构设计：App层负责连接各层
 * - 实现Modbus回调接口，调用emm_v5 API
 * - 在初始化时注册到Modbus模块
 * - 完成Middlewares层与BSP层的解耦
 * 
 ******************************************************************************
 */

#include "modbus_adapter.h"
#include "modbus_hal.h"
#include "emm_v5.h"

/* ======================== 回调函数实现 ======================== */

/**
 * @brief   电机使能控制回调
 */
static void modbus_cb_motor_enable(uint8_t addr, bool enable, bool sync)
{
    Emm_V5_En_Control(addr, enable, sync);
}

/**
 * @brief   电机位置控制回调
 */
static void modbus_cb_motor_pos_control(uint8_t addr, uint8_t dir, uint16_t speed, 
                                        uint8_t acc, uint32_t pulses, bool relative, bool sync)
{
    Emm_V5_Pos_Control(addr, dir, speed, acc, pulses, relative, sync);
}

/**
 * @brief   电机速度控制回调
 */
static void modbus_cb_motor_vel_control(uint8_t addr, uint8_t dir, uint16_t speed, 
                                        uint8_t acc, bool sync)
{
    Emm_V5_Vel_Control(addr, dir, speed, acc, sync);
}

/**
 * @brief   电机急停回调
 */
static void modbus_cb_motor_stop(uint8_t addr, bool sync)
{
    Emm_V5_Stop_Now(addr, sync);
}

/**
 * @brief   电机位置清零回调
 */
static void modbus_cb_motor_reset_position(uint8_t addr)
{
    Emm_V5_Reset_CurPos_To_Zero(addr);
}

/**
 * @brief   同步运动回调
 */
static void modbus_cb_motor_sync_motion(uint8_t addr)
{
    Emm_V5_Synchronous_motion(addr);
}

/**
 * @brief   读取电机状态回调（可选）
 * @note    V3.5 Phase 4: 暂时返回-1，实际状态由轮询机制更新
 */
static int modbus_cb_motor_read_status(uint8_t addr, uint32_t *position, uint16_t *speed)
{
    /* TODO: 实现电机状态读取，需要emm_v5提供查询接口 */
    (void)addr;
    (void)position;
    (void)speed;
    return -1;  /* 暂未实现 */
}

/**
 * @brief   电机回零回调（V3.5 Phase 4新增）
 */
static void modbus_cb_motor_home(uint8_t addr, uint8_t mode, bool sync)
{
    Emm_V5_Origin_Trigger_Return(addr, mode, sync);
}

/**
 * @brief   解除保护回调（V3.5 Phase 4新增）
 */
static void modbus_cb_motor_release(uint8_t addr)
{
    Emm_V5_Reset_Clog_Pro(addr);
}

/* ======================== 公共函数 ======================== */

/**
 * @brief       初始化Modbus适配器并注册回调
 * @param       无
 * @retval      0: 成功, -1: 失败
 */
int modbus_adapter_init(void)
{
    /* 构造回调函数结构体 */
    modbus_motor_callbacks_t callbacks = {
        .motor_enable = modbus_cb_motor_enable,
        .motor_pos_control = modbus_cb_motor_pos_control,
        .motor_vel_control = modbus_cb_motor_vel_control,
        .motor_stop = modbus_cb_motor_stop,
        .motor_reset_position = modbus_cb_motor_reset_position,
        .motor_sync_motion = modbus_cb_motor_sync_motion,
        .motor_read_status = modbus_cb_motor_read_status,
        .motor_home = modbus_cb_motor_home,          /* V3.5 Phase 4 */
        .motor_release = modbus_cb_motor_release,    /* V3.5 Phase 4 */
    };
    
    /* 注册到Modbus模块 */
    return modbus_register_motor_callbacks(&callbacks);
}
