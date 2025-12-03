/**
 ****************************************************************************************************
 * @file        printer_axis.h
 * @author      STM32_485 Project
 * @version     V1.0
 * @date        2025-12-03
 * @brief       3D打印机3轴（4电机）控制接口
 ****************************************************************************************************
 * @attention   硬件配置：
 *              - X轴(宽度): 1台电机, 地址0x01
 *              - Y轴(深度): 2台电机同步, 地址0x02+0x03
 *              - Z轴(高度): 1台电机, 地址0x04
 *              
 *              坐标系定义：
 *              - X轴正方向：向右 (增加宽度)
 *              - Y轴正方向：向前 (增加深度)
 *              - Z轴正方向：向上 (增加高度)
 ****************************************************************************************************
 */

#ifndef __PRINTER_AXIS_H
#define __PRINTER_AXIS_H

#include "app_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================================
 * 类型定义
 * ============================================================================ */

/* 轴枚举类型 */
typedef enum {
    AXIS_X = 0,     /* X轴（宽度） */
    AXIS_Y = 1,     /* Y轴（深度，双电机） */
    AXIS_Z = 2,     /* Z轴（高度） */
    AXIS_COUNT = 3
} printer_axis_t;

/* 轴状态结构体 */
typedef struct {
    bool enabled;           /* 轴是否使能 */
    bool homed;             /* 是否已回零 */
    float position_mm;      /* 当前位置（mm，相对于原点）*/
    int32_t position_pulses;/* 当前位置（脉冲数，用于电机控制）*/
    uint16_t speed;         /* 当前速度（RPM） */
    uint8_t acceleration;   /* 当前加速度 */
    uint32_t last_move_tick; /* 上次运动时间戳 */
} axis_state_t;

/* 3D打印机状态 */
typedef struct {
    axis_state_t axes[AXIS_COUNT];  /* 3个轴的状态 */
    bool all_homed;                 /* 所有轴是否已回零 */
    bool emergency_stop;            /* 急停标志 */
    uint32_t total_moves;           /* 总运动次数 */
} printer_state_t;

/* ============================================================================
 * 公共函数声明
 * ============================================================================ */

/**
 * @brief       初始化3D打印机轴控制系统
 * @param       无
 * @retval      无
 * @note        1. 初始化所有轴状态
 *              2. 如果配置了PRINTER_ENABLE_ALL_ON_START，自动使能所有电机
 *              3. 如果配置了PRINTER_HOME_ON_STARTUP，自动回零
 */
void printer_axis_init(void);

/**
 * @brief       使能指定轴
 * @param       axis: 轴枚举 (AXIS_X, AXIS_Y, AXIS_Z)
 * @retval      true=成功, false=失败
 */
bool printer_axis_enable(printer_axis_t axis);

/**
 * @brief       失能指定轴
 * @param       axis: 轴枚举
 * @retval      true=成功, false=失败
 */
bool printer_axis_disable(printer_axis_t axis);

/**
 * @brief       使能所有轴（广播命令）
 * @param       无
 * @retval      true=成功, false=失败
 */
bool printer_axis_enable_all(void);

/**
 * @brief       失能所有轴（广播命令）
 * @param       无
 * @retval      true=成功, false=失败
 */
bool printer_axis_disable_all(void);

/**
 * @brief       单轴相对运动
 * @param       axis: 轴枚举
 * @param       distance: 移动距离（脉冲数，正数=正方向，负数=负方向）
 * @param       speed: 速度(RPM), 0=使用默认速度
 * @param       acc: 加速度(0-255), 0=使用默认加速度
 * @retval      true=成功, false=失败
 */
bool printer_axis_move_relative(printer_axis_t axis, int32_t distance, uint16_t speed, uint8_t acc);

/**
 * @brief       单轴绝对运动
 * @param       axis: 轴枚举
 * @param       target_pos: 目标位置（脉冲数）
 * @param       speed: 速度(RPM), 0=使用默认速度
 * @param       acc: 加速度(0-255), 0=使用默认加速度
 * @retval      true=成功, false=失败
 */
bool printer_axis_move_absolute(printer_axis_t axis, int32_t target_pos, uint16_t speed, uint8_t acc);

/**
 * @brief       XYZ三轴同步运动（相对）
 * @param       x_dist: X轴移动距离（脉冲数）
 * @param       y_dist: Y轴移动距离（脉冲数）
 * @param       z_dist: Z轴移动距离（脉冲数）
 * @param       speed: 统一速度(RPM), 0=使用各轴默认速度
 * @retval      true=成功, false=失败
 * @note        使用Emm_V5同步运动功能，三轴同时启动
 */
bool printer_move_xyz_sync(int32_t x_dist, int32_t y_dist, int32_t z_dist, uint16_t speed);

/**
 * @brief       XY平面运动（Z不动）
 * @param       x_dist: X轴移动距离（脉冲数）
 * @param       y_dist: Y轴移动距离（脉冲数）
 * @param       speed: 速度(RPM), 0=使用默认速度
 * @retval      true=成功, false=失败
 */
bool printer_move_xy(int32_t x_dist, int32_t y_dist, uint16_t speed);

/* ============================================================================
 * 以毫米为单位的运动API（3D打印机应用层优先使用）
 * ============================================================================ */

/**
 * @brief       单轴移动（毫米单位）
 * @param       axis: 轴枚举 (AXIS_X/AXIS_Y/AXIS_Z)
 * @param       distance_mm: 移动距离（单位: mm，正负可选）
 * @param       speed: 速度(RPM), 0=使用默认速度
 * @param       acc: 加速度(0-255), 0=使用默认加速度
 * @retval      true=成功, false=失败
 * @note        自动转换mm到脉冲数，基于丝杠导程20mm/圈
 */
bool printer_move_mm(printer_axis_t axis, float distance_mm, uint16_t speed, uint8_t acc);

/**
 * @brief       XYZ三轴同步移动（毫米单位）
 * @param       x_mm: X轴移动距离（mm）
 * @param       y_mm: Y轴移动距离（mm）
 * @param       z_mm: Z轴移动距离（mm）
 * @param       speed: 统一速度(RPM), 0=使用各轴默认速度
 * @retval      true=成功, false=失败
 * @note        使用Emm_V5同步运动功能，三轴同时启动
 */
bool printer_move_xyz_mm(float x_mm, float y_mm, float z_mm, uint16_t speed);

/**
 * @brief       XY平面移动（毫米单位）
 * @param       x_mm: X轴移动距离（mm）
 * @param       y_mm: Y轴移动距离（mm）
 * @param       speed: 速度(RPM), 0=使用默认速度
 * @retval      true=成功, false=失败
 */
bool printer_move_xy_mm(float x_mm, float y_mm, uint16_t speed);

/**
 * @brief       获取轴当前位置（毫米单位）
 * @param       axis: 轴枚举
 * @retval      当前位置（mm）
 */
float printer_get_position_mm(printer_axis_t axis);

/**
 * @brief       指定轴回零
 * @param       axis: 轴枚举
 * @param       mode: 回零模式 (0=单圈就近, 1=单圈方向, 2=多圈无限位, 3=多圈限位)
 * @retval      true=成功, false=失败
 */
bool printer_axis_home(printer_axis_t axis, uint8_t mode);

/**
 * @brief       所有轴依次回零（Z→Y→X顺序）
 * @param       mode: 回零模式
 * @retval      true=成功, false=失败
 * @note        推荐顺序：先Z轴抬起，避免碰撞
 */
bool printer_home_all(uint8_t mode);

/**
 * @brief       紧急停止所有轴
 * @param       无
 * @retval      true=成功, false=失败
 */
bool printer_emergency_stop(void);

/**
 * @brief       获取轴状态
 * @param       axis: 轴枚举
 * @retval      指向轴状态的指针
 */
const axis_state_t* printer_get_axis_state(printer_axis_t axis);

/**
 * @brief       获取打印机整体状态
 * @param       无
 * @retval      指向打印机状态的指针
 */
const printer_state_t* printer_get_state(void);

/**
 * @brief       设置轴位置（手动校准用）
 * @param       axis: 轴枚举
 * @param       position: 位置值（脉冲数）
 * @retval      无
 */
void printer_set_axis_position(printer_axis_t axis, int32_t position);

/**
 * @brief       清除急停状态
 * @param       无
 * @retval      无
 */
void printer_clear_emergency_stop(void);

#endif /* __PRINTER_AXIS_H */
