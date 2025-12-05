/**
 ****************************************************************************************************
 * @file        motor_sequence.h
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-05
 * @brief       电机运动序列控制模块头文件
 ****************************************************************************************************
 * @attention   功能说明：
 *              - 实现预定义的电机运动序列
 *              - 支持按键触发运动序列
 *              - 提供非阻塞的序列执行状态查询
 ****************************************************************************************************
 */

#ifndef __MOTOR_SEQUENCE_H
#define __MOTOR_SEQUENCE_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 运动序列定义
 * ============================================================================ */

/* 序列步骤枚举 */
typedef enum {
    SEQ_STEP_IDLE = 0,      /* 空闲状态 */
    SEQ_STEP_FORWARD_1,     /* 第一步：前进5.12mm */
    SEQ_STEP_BACKWARD,      /* 第二步：后退10.22mm */
    SEQ_STEP_FORWARD_2,     /* 第三步：前进15.32mm */
    SEQ_STEP_STOP,          /* 第四步：停止 */
    SEQ_STEP_COMPLETE       /* 序列完成 */
} sequence_step_t;

/* 序列状态结构体 */
typedef struct {
    sequence_step_t current_step;    /* 当前执行步骤 */
    bool is_running;                 /* 序列是否正在运行 */
    uint32_t start_time;             /* 序列开始时间 */
    uint32_t step_start_time;        /* 当前步骤开始时间 */
    uint32_t step_delay_ms;          /* 当前步骤延迟时间 */
    int32_t target_position;         /* 当前步骤目标位置（脉冲数） */
    uint16_t speed;                  /* 运动速度 (RPM) */
    uint8_t motor_addr;              /* 电机地址 */
} sequence_state_t;

/* ============================================================================
 * 公共接口函数
 * ============================================================================ */

/**
 * @brief       运动序列模块初始化
 * @param       无
 * @retval      无
 */
void motor_sequence_init(void);

/**
 * @brief       执行X轴运动序列（按键触发）
 *              序列：前进5.12mm → 后退10.22mm → 前进15.32mm → 停止
 *              使用0.02mm精度单位：5.12mm=256单位, 10.22mm=511单位, 15.32mm=766单位
 * @param       无
 * @retval      true=序列开始执行，false=序列已在运行
 */
bool motor_sequence_run_x_axis_demo(void);

/**
 * @brief       查询运动序列执行状态
 * @param       无
 * @retval      当前序列状态结构体指针
 */
const sequence_state_t* motor_sequence_get_status(void);

/**
 * @brief       强制停止当前运动序列
 * @param       无
 * @retval      无
 */
void motor_sequence_stop(void);

/**
 * @brief       运动序列主循环任务（需在主循环中调用）
 * @param       无
 * @retval      无
 * @note        处理序列状态转换和运动控制
 */
void motor_sequence_task(void);

#endif /* __MOTOR_SEQUENCE_H */