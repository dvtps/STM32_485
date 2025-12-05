/**
 ****************************************************************************************************
 * @file        motor_sequence.c
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-05
 * @brief       电机运动序列控制模块实现
 ****************************************************************************************************
 * @attention   实现说明：
 *              - 使用状态机管理运动序列
 *              - 基于时间延迟的步骤切换（简化实现）
 *              - 支持非阻塞查询和强制停止
 ****************************************************************************************************
 */

#include "motor_sequence.h"
#include "y_v2.h"                    /* 电机控制API */
#include "app_config.h"              /* 系统配置 */
#include "emm_v5_parser.h"           /* 响应帧解析 */
#include "emm_uart.h"                /* UART通信 */
#include <stdio.h>
#include <stdlib.h>                  /* for abs() */

/* ============================================================================
 * 私有定义
 * ============================================================================ */

/* 运动参数定义 */
#define SEQ_MOTOR_ADDR          0x01    /* X轴电机地址 */
#define SEQ_SPEED_RPM           300     /* 运动速度：300 RPM */
#define SEQ_ACCELERATION        50      /* 加速度：50 (适当增加加速度以减少加速时间) */

/* 距离转换为脉冲数 (160脉冲/mm) */
#define DISTANCE_TO_PULSES(mm)  ((uint32_t)((mm) * PULSES_PER_MM + 0.5f))

/* 序列距离定义 (0.02mm整数单位) */
#define SEQ_FORWARD_1_UNITS     256     /* 第一步前进距离：256 * 0.02mm = 5.12mm */
#define SEQ_BACKWARD_UNITS      511     /* 第二步后退距离：511 * 0.02mm = 10.22mm */
#define SEQ_FORWARD_2_UNITS     766     /* 第三步前进距离：766 * 0.02mm = 15.32mm */

/* 单位转换：0.02mm单位转换为毫米 */
#define UNITS_TO_MM(units)      ((float)(units) * 0.02f)
#define MM_TO_UNITS(mm)         ((int32_t)((mm) / 0.02f + 0.5f))

/* 步骤执行时间 (毫秒) - 根据电机速度和距离动态估算 */
#define SEQ_STEP_DELAY_MS       300     /* 缩短延迟时间，从1000ms改为300ms */

/* 到位检测参数 */
#define POSITION_TOLERANCE_PULSES  10      /* 位置误差容限：10脉冲（约0.0625mm） */
#define POSITION_QUERY_INTERVAL_MS 50      /* 位置查询间隔：50ms */
#define MAX_POSITION_WAIT_MS       5000    /* 最大等待时间：5秒 */

/* 位置查询状态 */
static struct {
    uint32_t last_query_time;        /* 上次查询时间 */
    bool waiting_for_response;       /* 等待响应中 */
    int32_t current_position;        /* 当前位置缓存 */
    bool position_valid;             /* 位置数据是否有效 */
} g_position_query = {0};

/* ============================================================================
 * 私有变量
 * ============================================================================ */

static sequence_state_t g_sequence_state = {
    .current_step = SEQ_STEP_IDLE,
    .is_running = false,
    .start_time = 0,
    .step_start_time = 0,
    .step_delay_ms = 0,
    .target_position = 0,
    .speed = SEQ_SPEED_RPM,
    .motor_addr = SEQ_MOTOR_ADDR
};

/* ============================================================================
 * 私有函数
 * ============================================================================ */

/**
 * @brief       计算运动时间估算（毫秒）
 * @param       distance_mm: 距离（毫米）
 * @param       speed_rpm: 速度（RPM）
 * @retval      估算时间（毫秒）
 * @note        基于线速度计算：线速度 = RPM/60 * 导程
 *              时间 = 距离/线速度 + 加速时间裕量
 */
static uint32_t calculate_move_time(float distance_mm, uint16_t speed_rpm)
{
    /* 线速度计算：RPM/60 * 20mm/圈 = mm/秒 */
    float linear_speed_mm_per_sec = (speed_rpm / 60.0f) * LEAD_SCREW_PITCH_MM;

    /* 基础运动时间 = 距离/速度 */
    uint32_t base_time_ms = (uint32_t)((distance_mm / linear_speed_mm_per_sec) * 1000.0f);

    /* 加上加速时间裕量（约20%）和最小延迟（50ms）*/
    uint32_t margin_time_ms = base_time_ms / 5;  /* 20%裕量 */
    uint32_t min_delay_ms = 50;  /* 最小延迟 */

    uint32_t total_time_ms = base_time_ms + margin_time_ms;
    if (total_time_ms < min_delay_ms) {
        total_time_ms = min_delay_ms;
    }

    return total_time_ms;
}

/**
 * @brief       查询电机当前位置
 * @param       无
 * @retval      无
 * @note        发送编码器位置查询命令（S_ENCL），非阻塞
 */
static void query_current_position(void)
{
    uint32_t current_time = HAL_GetTick();
    
    /* 检查是否需要查询位置 */
    if (g_position_query.waiting_for_response || 
        (current_time - g_position_query.last_query_time) < POSITION_QUERY_INTERVAL_MS) {
        return;
    }
    
    /* 发送编码器位置查询命令（使用S_ENCL获得更准确的位置） */
    Y_V2_Read_Sys_Params(SEQ_MOTOR_ADDR, S_ENCL);
    g_position_query.last_query_time = current_time;
    g_position_query.waiting_for_response = true;
    g_position_query.position_valid = false;
}

/**
 * @brief       处理位置查询响应
 * @param       position: 查询到的位置值
 * @retval      无
 */
static void handle_position_response(int32_t position)
{
    g_position_query.current_position = position;
    g_position_query.position_valid = true;
    g_position_query.waiting_for_response = false;
}

/**
 * @brief       检查电机是否运动到位
 * @param       无
 * @retval      true=已到位，false=未到位
 */
static bool is_motor_in_position(void)
{
    /* 如果位置数据无效，返回false */
    if (!g_position_query.position_valid) {
        return false;
    }
    
    /* 计算位置误差 */
    int32_t position_error = abs(g_sequence_state.target_position - g_position_query.current_position);
    
    /* 检查是否在容限范围内 */
    return (position_error <= POSITION_TOLERANCE_PULSES);
}

/**
 * @brief       执行单步运动
 * @param       direction: 方向 (0=CW顺时针, 1=CCW逆时针)
 * @param       distance_mm: 距离 (毫米)
 * @retval      估算的运动时间（毫秒）
 */
static uint32_t execute_step(uint8_t direction, float distance_mm)
{
    uint32_t pulses = DISTANCE_TO_PULSES(distance_mm);
    uint32_t estimated_time_ms = calculate_move_time(distance_mm, SEQ_SPEED_RPM);

    printf("[SEQ] 执行步骤: %s %.2fmm (%lu脉冲), 速度=%dRPM, 估算时间=%lums\r\n",
           (direction == 0) ? "前进" : "后退",
           distance_mm, (unsigned long)pulses, SEQ_SPEED_RPM, (unsigned long)estimated_time_ms);

    /* 计算并记录目标位置 */
    int32_t direction_sign = (direction == 0) ? 1 : -1;
    g_sequence_state.target_position += (int32_t)pulses * direction_sign;

    /* 发送运动控制API */
    Y_V2_Traj_Pos_Control(
        SEQ_MOTOR_ADDR,           /* 电机地址 */
        direction,                /* 方向: 0=CW, 1=CCW */
        SEQ_ACCELERATION,         /* 加速度 */
        SEQ_ACCELERATION,         /* 减速度 */
        SEQ_SPEED_RPM,            /* 速度 (RPM) */
        distance_mm,              /* 距离 (mm) */
        0,                        /* 相对模式 */
        false                     /* 不等待同步 */
    );

    /* 重置位置查询状态 */
    g_position_query.position_valid = false;
    g_position_query.waiting_for_response = false;

    return estimated_time_ms;
}

/* ============================================================================
 * 公共接口函数实现
 * ============================================================================ */

/**
 * @brief       运动序列模块初始化
 * @param       无
 * @retval      无
 */
void motor_sequence_init(void)
{
    /* 初始化序列状态 */
    g_sequence_state.current_step = SEQ_STEP_IDLE;
    g_sequence_state.is_running = false;
    g_sequence_state.start_time = 0;
    g_sequence_state.step_start_time = 0;
    g_sequence_state.step_delay_ms = 0;
    g_sequence_state.target_position = 0;

    /* 初始化位置查询状态 */
    g_position_query.last_query_time = 0;
    g_position_query.waiting_for_response = false;
    g_position_query.current_position = 0;
    g_position_query.position_valid = false;

    printf("[SEQ] 运动序列模块已初始化\r\n");
}

/**
 * @brief       执行X轴运动序列（按键触发）
 *              序列：前进5.12mm → 后退10.22mm → 前进15.32mm → 停止
 * @param       无
 * @retval      true=序列开始执行，false=序列已在运行
 */
bool motor_sequence_run_x_axis_demo(void)
{
    /* 检查是否已在运行 */
    if (g_sequence_state.is_running) {
        printf("[SEQ] 序列已在运行中，请等待完成\r\n");
        return false;
    }

    /* 初始化序列状态 */
    g_sequence_state.current_step = SEQ_STEP_FORWARD_1;
    g_sequence_state.is_running = true;
    g_sequence_state.start_time = HAL_GetTick();

    printf("[SEQ] 开始执行X轴运动序列: 前进5.12mm → 后退10.22mm → 前进15.32mm → 停止\r\n");

    /* 执行第一步：前进5.12mm，记录估算时间 */
    uint32_t step_time = execute_step(0, UNITS_TO_MM(SEQ_FORWARD_1_UNITS));  /* 0=CW前进 */
    g_sequence_state.step_start_time = HAL_GetTick();
    g_sequence_state.step_delay_ms = step_time;

    return true;
}

/**
 * @brief       查询运动序列执行状态
 * @param       无
 * @retval      当前序列状态结构体指针
 */
const sequence_state_t* motor_sequence_get_status(void)
{
    return &g_sequence_state;
}

/**
 * @brief       强制停止当前运动序列
 * @param       无
 * @retval      无
 */
void motor_sequence_stop(void)
{
    if (g_sequence_state.is_running) {
        /* 发送停止命令 */
        Y_V2_Stop_Now(SEQ_MOTOR_ADDR, false);

        /* 重置状态 */
        g_sequence_state.current_step = SEQ_STEP_IDLE;
        g_sequence_state.is_running = false;

        printf("[SEQ] 运动序列已强制停止\r\n");
    }
}

/**
 * @brief       运动序列主循环任务（需在主循环中调用）
 * @param       无
 * @retval      无
 * @note        处理序列状态转换和运动控制
 */
void motor_sequence_task(void)
{
    /* 如果序列未运行，直接返回 */
    if (!g_sequence_state.is_running) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    /* 检查是否超时（防止电机卡死） */
    uint32_t step_elapsed = current_time - g_sequence_state.step_start_time;
    if (step_elapsed > MAX_POSITION_WAIT_MS) {
        printf("[SEQ] 运动超时！强制停止序列\r\n");
        motor_sequence_stop();
        return;
    }

    /* 查询当前位置（非阻塞） */
    query_current_position();

    /* 处理位置响应（如果有的话） */
    extern uint8_t g_emm_rx_cmd[];
    extern uint16_t g_emm_rx_count;
    extern volatile uint8_t g_emm_frame_complete;

    if (g_emm_frame_complete && g_emm_rx_count > 0) {
        /* 解析位置响应 */
        int32_t position;
        if (emm_parser_position(g_emm_rx_cmd, g_emm_rx_count, &position)) {
            handle_position_response(position);
        }
        g_emm_frame_complete = 0;  /* 清除帧完成标志 */
    }

    /* 检查当前步骤是否到位 */
    if (is_motor_in_position()) {
        /* 当前步骤到位，切换到下一步 */
        printf("[SEQ] 步骤到位，切换到下一步\r\n");

        switch (g_sequence_state.current_step) {
            case SEQ_STEP_FORWARD_1:
                /* 第一步完成，执行第二步：后退10.22mm */
                g_sequence_state.current_step = SEQ_STEP_BACKWARD;
                execute_step(1, UNITS_TO_MM(SEQ_BACKWARD_UNITS));  /* 1=CCW后退 */
                g_sequence_state.step_start_time = current_time;
                break;

            case SEQ_STEP_BACKWARD:
                /* 第二步完成，执行第三步：前进15.32mm */
                g_sequence_state.current_step = SEQ_STEP_FORWARD_2;
                execute_step(0, UNITS_TO_MM(SEQ_FORWARD_2_UNITS));  /* 0=CW前进 */
                g_sequence_state.step_start_time = current_time;
                break;

            case SEQ_STEP_FORWARD_2:
                /* 第三步完成，执行第四步：停止 */
                g_sequence_state.current_step = SEQ_STEP_STOP;
                Y_V2_Stop_Now(SEQ_MOTOR_ADDR, false);
                printf("[SEQ] 运动序列执行完成\r\n");
                HAL_Delay(200);  /* 短暂延迟确保停止命令发送 */
                g_sequence_state.current_step = SEQ_STEP_COMPLETE;
                g_sequence_state.is_running = false;
                printf("[SEQ] 运动序列全部完成\r\n");
                break;

            default:
                /* 异常状态，重置 */
                g_sequence_state.current_step = SEQ_STEP_IDLE;
                g_sequence_state.is_running = false;
                printf("[SEQ] 运动序列异常，重置到空闲状态\r\n");
                break;
        }
    }
}