/**
 ****************************************************************************************************
 * @file        motor_monitor.h
 * @author      STM32_485 Project Team
 * @version     V3.7
 * @date        2025-12-03
 * @brief       电机监控与反馈系统（非阻塞异步架构）
 ****************************************************************************************************
 * @attention   设计原则：
 *              1. 非阻塞轮询：主循环定期调用，不影响实时性
 *              2. 在线检测：自动标记离线电机（Y/Z未连接）
 *              3. 位置同步：周期查询编码器真实位置
 *              4. 异常告警：堵转/失步/超时自动检测
 ****************************************************************************************************
 */

#ifndef __MOTOR_MONITOR_H
#define __MOTOR_MONITOR_H

#include "stm32f1xx_hal.h"
#include "emm_v5_parser.h"
#include <stdbool.h>

/* ======================== 配置参数 ======================== */

#define MOTOR_MONITOR_MAX_MOTORS    4           /* 最大支持电机数量 */
#define MOTOR_ONLINE_TIMEOUT_MS     1000        /* 在线超时（1秒无响应=离线）*/
#define MOTOR_POS_QUERY_PERIOD_MS   200         /* 位置查询周期（200ms）*/
#define MOTOR_FLAG_QUERY_PERIOD_MS  500         /* 状态查询周期（500ms）*/
#define MOTOR_OFFLINE_RETRY_PERIOD_MS  30000    /* 离线电机重试周期（30秒）*/
#define MOTOR_POS_ERROR_THRESHOLD   100         /* 位置误差阈值（100脉冲）*/

/* ======================== 数据结构定义 ======================== */

/**
 * @brief       电机监控状态
 */
typedef struct {
    /* 基本信息 */
    uint8_t addr;                   /* 电机地址（1-255）*/
    bool enabled;                   /* 监控使能（false=跳过此电机）*/
    
    /* 在线状态 */
    bool online;                    /* 在线状态（true=在线，false=离线）*/
    bool status_determined;         /* 状态已确定（首次查询完成后设置，停止轮询）*/
    uint32_t last_response_tick;    /* 最后响应时间戳（HAL_GetTick）*/
    uint16_t timeout_count;         /* 连续超时次数 */
    
    /* 位置信息 */
    int32_t cmd_position;           /* 命令位置（本地累积）*/
    int32_t real_position;          /* 真实位置（编码器反馈）*/
    int32_t position_error;         /* 位置误差（cmd - real）*/
    bool position_synced;           /* 位置已同步标志 */
    
    /* 速度信息 */
    int16_t real_velocity;          /* 实时速度（RPM）*/
    
    /* 状态标志 */
    motor_flag_t status;            /* 电机状态标志（S_FLAG）*/
    
    /* 查询控制 */
    uint32_t last_pos_query_tick;   /* 上次位置查询时间 */
    uint32_t last_flag_query_tick;  /* 上次状态查询时间 */
    bool waiting_pos_response;      /* 等待位置响应中 */
    bool waiting_flag_response;     /* 等待状态响应中 */
    
    /* 统计信息 */
    uint32_t query_success_count;   /* 查询成功次数 */
    uint32_t query_timeout_count;   /* 查询超时次数 */
    uint32_t clog_detected_count;   /* 堵转检测次数 */
} motor_monitor_t;

/* ======================== 公共函数声明 ======================== */

/**
 * @brief       初始化电机监控系统
 * @param       无
 * @retval      无
 */
void motor_monitor_init(void);

/**
 * @brief       注册需要监控的电机
 * @param       addr: 电机地址（1-255）
 * @retval      true=注册成功，false=失败（已满或地址无效）
 */
bool motor_monitor_register(uint8_t addr);

/**
 * @brief       注销电机监控
 * @param       addr: 电机地址
 * @retval      无
 */
void motor_monitor_unregister(uint8_t addr);

/**
 * @brief       主循环任务：电机监控（非阻塞）
 * @param       无
 * @retval      无
 * @note        ⚡ 在app_tasks_run()中调用，每次主循环执行
 *              - 自动发送位置/状态查询（按周期）
 *              - 处理接收到的响应帧
 *              - 更新在线状态和位置信息
 */
void motor_monitor_task(void);

/**
 * @brief       处理接收到的响应帧（由app_tasks调用）
 * @param       rx_data: 响应帧数据
 * @param       len: 数据长度
 * @retval      true=处理成功，false=非监控电机或解析失败
 * @note        此函数解析响应并更新对应电机的状态
 */
bool motor_monitor_process_response(const uint8_t *rx_data, uint16_t len);

/**
 * @brief       更新命令位置（发送运动命令后调用）
 * @param       addr: 电机地址
 * @param       delta: 位置增量（相对运动的脉冲数，可为负）
 * @retval      无
 * @note        调用时机：printer_axis_move_relative()发送命令后
 */
void motor_monitor_update_cmd_position(uint8_t addr, int32_t delta);

/**
 * @brief       设置命令位置（绝对运动后调用）
 * @param       addr: 电机地址
 * @param       position: 绝对位置
 * @retval      无
 */
void motor_monitor_set_cmd_position(uint8_t addr, int32_t position);

/**
 * @brief       获取电机在线状态
 * @param       addr: 电机地址
 * @retval      true=在线，false=离线或未注册
 */
bool motor_monitor_is_online(uint8_t addr);

/**
 * @brief       获取电机真实位置（编码器反馈）
 * @param       addr: 电机地址
 * @param       position: 位置输出（调用者分配）
 * @retval      true=获取成功，false=电机离线或未同步
 */
bool motor_monitor_get_real_position(uint8_t addr, int32_t *position);

/**
 * @brief       获取电机位置误差
 * @param       addr: 电机地址
 * @param       error: 误差输出（调用者分配）
 * @retval      true=获取成功，false=电机离线
 */
bool motor_monitor_get_position_error(uint8_t addr, int32_t *error);

/**
 * @brief       获取电机状态标志
 * @param       addr: 电机地址
 * @param       flag: 状态标志输出（调用者分配）
 * @retval      true=获取成功，false=电机离线
 */
bool motor_monitor_get_flag(uint8_t addr, motor_flag_t *flag);

/**
 * @brief       获取电机监控信息（调试用）
 * @param       addr: 电机地址
 * @retval      电机监控结构体指针（只读），NULL=未注册
 */
const motor_monitor_t* motor_monitor_get_info(uint8_t addr);

/**
 * @brief       打印所有电机监控状态（调试用）
 * @param       无
 * @retval      无
 */
void motor_monitor_print_status(void);

/**
 * @brief       强制查询电机位置（立即发送查询命令）
 * @param       addr: 电机地址
 * @retval      true=命令发送成功，false=电机未注册
 * @note        正常情况下不需要手动调用，由motor_monitor_task()自动处理
 */
bool motor_monitor_query_position_now(uint8_t addr);

/**
 * @brief       强制查询电机状态标志（立即发送查询命令）
 * @param       addr: 电机地址
 * @retval      true=命令发送成功，false=电机未注册
 */
bool motor_monitor_query_flag_now(uint8_t addr);

#endif /* __MOTOR_MONITOR_H */
