/**
 ****************************************************************************************************
 * @file        motor_monitor.c
 * @author      STM32_485 Project Team
 * @version     V3.7
 * @date        2025-12-03
 * @brief       电机监控与反馈系统实现（高性能非阻塞）
 ****************************************************************************************************
 */

#include "motor_monitor.h"
#include "y_v2.h"            /* V3.0: Y系X固件协议驱动 */
#include "emm_v5_parser.h"  /* 响应帧解析器(兼容Y_V2) */
#include "logger.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ======================== 私有变量 ======================== */

static motor_monitor_t g_motors[MOTOR_MONITOR_MAX_MOTORS] = {0};
static uint8_t g_motor_count = 0;
static bool g_all_motors_checked = false;  /* 所有电机状态已确定 */

/* 轴名称映射（与printer_axis.h对应）*/
static const char* get_axis_name(uint8_t addr) {
    switch (addr) {
        case 0x01: return "X轴";
        case 0x02: return "Y轴左";
        case 0x03: return "Y轴右";
        case 0x04: return "Z轴";
        default: return "未知";
    }
}

/* ======================== 私有函数声明 ======================== */

static motor_monitor_t* find_motor_by_addr(uint8_t addr);
static void handle_position_response(motor_monitor_t *motor, int32_t position);
static void handle_flag_response(motor_monitor_t *motor, motor_flag_t *flag);
static void check_motor_timeout(motor_monitor_t *motor);

/* ======================== 公共函数实现 ======================== */

/**
 * @brief       初始化电机监控系�?
 */
void motor_monitor_init(void)
{
    memset(g_motors, 0, sizeof(g_motors));
    g_motor_count = 0;
    
    /* 初始化解析器 */
    emm_parser_init();
    
    LOG_MONITOR("电机监控系统已初始化");
}

/**
 * @brief       注册需要监控的电机
 */
bool motor_monitor_register(uint8_t addr)
{
    if (addr == 0 || g_motor_count >= MOTOR_MONITOR_MAX_MOTORS) {
        return false;
    }
    
    /* 检查是否已注册 */
    if (find_motor_by_addr(addr) != NULL) {
        return true;  /* 已存�?*/
    }
    
    /* 添加新电�?*/
    motor_monitor_t *motor = &g_motors[g_motor_count++];
    memset(motor, 0, sizeof(motor_monitor_t));
    
    motor->addr = addr;
    motor->enabled = true;
    motor->online = false;  /* 初始假设离线，等待首次响�?*/
    motor->position_synced = false;
    motor->last_response_tick = HAL_GetTick();  /* 设置初始时间戳，用于超时检�?*/
    
    LOG_MONITOR("已注册电机 0x%02X", addr);
    return true;
}

/**
 * @brief       注销电机监控
 */
void motor_monitor_unregister(uint8_t addr)
{
    for (uint8_t i = 0; i < g_motor_count; i++) {
        if (g_motors[i].addr == addr) {
            /* 后续元素前移 */
            for (uint8_t j = i; j < g_motor_count - 1; j++) {
                g_motors[j] = g_motors[j + 1];
            }
            g_motor_count--;
            printf("[MONITOR] Unregistered motor 0x%02X\r\n", addr);
            return;
        }
    }
}

/**
 * @brief       主循环任务：电机监控（非阻塞�?
 * @note        �?关键：此函数执行时间<1ms，不影响实时�?
 */
void motor_monitor_task(void)
{
    uint32_t now = HAL_GetTick();
    
    for (uint8_t i = 0; i < g_motor_count; i++) {
        motor_monitor_t *motor = &g_motors[i];
        
        if (!motor->enabled) {
            continue;
        }
        
        /* �?一次性查询模式：状态已确定的电机停止轮�?*/
        if (motor->status_determined) {
            continue;  /* 跳过已确定在�?离线状态的电机 */
        }
        
        /* 1. 检查超时（在线状态更新）*/
        check_motor_timeout(motor);
        
        /* 关键优化：离线电机大幅降低轮询频率（调试/组装状态）*/
        if (!motor->online && motor->query_timeout_count > 3) {
            /* 离线电机降级10秒轮询一次（检测是否重新上线）*/
            if (now - motor->last_pos_query_tick < MOTOR_OFFLINE_RETRY_PERIOD_MS) {
                continue;  /* 跳过此电机的查询 */
            }
        }
        
        /* 2. 周期查询位置（200ms）*/
        if (!motor->waiting_pos_response && 
            (now - motor->last_pos_query_tick >= MOTOR_POS_QUERY_PERIOD_MS)) {
            
            motor->last_pos_query_tick = now;
            motor->waiting_pos_response = true;
            
            /* 发送位置查询命令（S_CPOS）*/
            Y_V2_Read_Sys_Params(motor->addr, S_CPOS);
        }
        
        /* 3. 周期查询状态（500ms）*/
        if (!motor->waiting_flag_response && 
            (now - motor->last_flag_query_tick >= MOTOR_FLAG_QUERY_PERIOD_MS)) {
            
            motor->last_flag_query_tick = now;
            motor->waiting_flag_response = true;
            
            /* 发送状态查询命令（S_FLAG）*/
            Y_V2_Read_Sys_Params(motor->addr, S_FLAG);
        }
    }
    
    /* 检查所有电机状态是否已确定 */
    if (!g_all_motors_checked) {
        bool all_checked = true;
        for (uint8_t i = 0; i < g_motor_count; i++) {
            if (g_motors[i].enabled && !g_motors[i].status_determined) {
                all_checked = false;
                break;
            }
        }
        
        if (all_checked) {
            g_all_motors_checked = true;
            printf("\r\n========================================\r\n");
            printf("         电机状态检测完成        \r\n");
            printf("========================================\r\n");
            for (uint8_t i = 0; i < g_motor_count; i++) {
                motor_monitor_t *m = &g_motors[i];
                if (m->enabled) {
                    printf("[%s][电机] 地址0x%02X %s\r\n", 
                           get_axis_name(m->addr), 
                           m->addr, 
                           m->online ? "已上线" : "离线");
                }
            }
            printf("========================================\r\n\r\n");
        }
    }
}

/**
 * @brief       处理接收到的响应帧
 */
bool motor_monitor_process_response(const uint8_t *rx_data, uint16_t len)
{
    if (!rx_data || len < 3) {
        return false;
    }
    
    /* 解析响应帧 */
    motor_response_t response;
    if (!emm_parser_parse(rx_data, len, &response)) {
        return false;  /* 解析失败 */
    }
    
    /* 查找对应电机 */
    motor_monitor_t *motor = find_motor_by_addr(response.motor_addr);
    if (!motor) {
        return false;  /* 非监控电机 */
    }

    /* 检查是否重新上线 */
    if (!motor->online) {
        printf("[MONITOR] Motor 0x%02X 重新上线, tick=%lu\r\n", motor->addr, (unsigned long)response.timestamp);
    }

    /* 更新在线状态 */
    motor->online = true;
    motor->last_response_tick = response.timestamp;
    motor->timeout_count = 0;
    motor->query_success_count++;

    /* 首次确认在线状态后停止查询（静默记录）*/
    if (!motor->status_determined) {
        motor->status_determined = true;
        /* 延迟打印，等待所有电机查询完成 */
    }
    
    /* 根据响应类型处理数据 */
    switch (response.type) {
        case EMM_RESP_POSITION:
            handle_position_response(motor, response.data.position);
            motor->waiting_pos_response = false;
            break;
            
        case EMM_RESP_FLAG:
            handle_flag_response(motor, &response.data.flag);
            motor->waiting_flag_response = false;
            break;
            
        case EMM_RESP_VELOCITY:
            motor->real_velocity = response.data.velocity;
            break;
            
        case EMM_RESP_PERR:
            motor->position_error = response.data.pos_error;
            break;
            
        default:
            break;
    }
    
    return true;
}

/**
 * @brief       更新命令位置
 */
void motor_monitor_update_cmd_position(uint8_t addr, int32_t delta)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    if (motor) {
        motor->cmd_position += delta;
    }
}

/**
 * @brief       设置命令位置
 */
void motor_monitor_set_cmd_position(uint8_t addr, int32_t position)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    if (motor) {
        motor->cmd_position = position;
    }
}

/**
 * @brief       获取电机在线状态
 */
bool motor_monitor_is_online(uint8_t addr)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    return (motor && motor->online);
}

/**
 * @brief       获取电机真实位置
 */
bool motor_monitor_get_real_position(uint8_t addr, int32_t *position)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    if (!motor || !motor->online || !motor->position_synced) {
        return false;
    }
    
    if (position) {
        *position = motor->real_position;
    }
    return true;
}

/**
 * @brief       获取电机位置误差
 */
bool motor_monitor_get_position_error(uint8_t addr, int32_t *error)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    if (!motor || !motor->online) {
        return false;
    }
    
    if (error) {
        *error = motor->position_error;
    }
    return true;
}

/**
 * @brief       获取电机状态标志
 */
bool motor_monitor_get_flag(uint8_t addr, motor_flag_t *flag)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    if (!motor || !motor->online) {
        return false;
    }
    
    if (flag) {
        *flag = motor->status;
    }
    return true;
}

/**
 * @brief       获取电机监控信息
 */
const motor_monitor_t* motor_monitor_get_info(uint8_t addr)
{
    return find_motor_by_addr(addr);
}

/**
 * @brief       打印所有电机监控状态
 */
void motor_monitor_print_status(void)
{
    printf("\r\n========== Motor Monitor Status ==========\r\n");
    printf("Registered motors: %u\r\n\r\n", g_motor_count);
    
    for (uint8_t i = 0; i < g_motor_count; i++) {
        motor_monitor_t *motor = &g_motors[i];
        
        printf("Motor 0x%02X:\r\n", motor->addr);
        printf("  Online: %s\r\n", motor->online ? "YES" : "NO");
        
        if (motor->online) {
            printf("  Cmd Pos: %ld\r\n", (long)motor->cmd_position);
            printf("  Real Pos: %ld%s\r\n", (long)motor->real_position, 
                   motor->position_synced ? "" : " (not synced)");
            printf("  Error: %ld\r\n", (long)motor->position_error);
            printf("  Velocity: %d RPM\r\n", motor->real_velocity);
            printf("  Enabled: %s\r\n", motor->status.motor_enabled ? "YES" : "NO");
            printf("  Homed: %s\r\n", motor->status.home_done ? "YES" : "NO");
            printf("  Clogged: %s\r\n", motor->status.is_clogged ? "YES" : "NO");
            printf("  Success: %lu, Timeout: %lu\r\n", 
                   (unsigned long)motor->query_success_count, 
                   (unsigned long)motor->query_timeout_count);
        } else {
            printf("  Offline for %lu ms\r\n", 
                   (unsigned long)(HAL_GetTick() - motor->last_response_tick));
        }
        printf("\r\n");
    }
    
    printf("==========================================\r\n");
}

/**
 * @brief       强制查询电机位置
 */
bool motor_monitor_query_position_now(uint8_t addr)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    if (!motor) {
        return false;
    }
    
    motor->waiting_pos_response = true;
    motor->last_pos_query_tick = HAL_GetTick();
    Y_V2_Read_Sys_Params(addr, S_CPOS);
    
    return true;
}

/**
 * @brief       强制查询电机状态标志
 */
bool motor_monitor_query_flag_now(uint8_t addr)
{
    motor_monitor_t *motor = find_motor_by_addr(addr);
    if (!motor) {
        return false;
    }
    
    motor->waiting_flag_response = true;
    motor->last_flag_query_tick = HAL_GetTick();
    Y_V2_Read_Sys_Params(addr, S_FLAG);
    
    return true;
}

/* ======================== 私有函数实现 ======================== */

/**
 * @brief       根据地址查找电机
 */
static motor_monitor_t* find_motor_by_addr(uint8_t addr)
{
    for (uint8_t i = 0; i < g_motor_count; i++) {
        if (g_motors[i].addr == addr) {
            return &g_motors[i];
        }
    }
    return NULL;
}

/**
 * @brief       处理位置响应
 */
static void handle_position_response(motor_monitor_t *motor, int32_t position)
{
    motor->real_position = position;
    motor->position_synced = true;
    
    /* 计算位置误差 */
    motor->position_error = motor->cmd_position - motor->real_position;
    
    /* 检查位置误差是否过大 */
    if (motor->position_error > MOTOR_POS_ERROR_THRESHOLD || 
        motor->position_error < -MOTOR_POS_ERROR_THRESHOLD) {
        printf("[WARN] Motor 0x%02X: Large pos error %ld\r\n", 
               motor->addr, (long)motor->position_error);
    }
}

/**
 * @brief       处理状态标志响应
 */
static void handle_flag_response(motor_monitor_t *motor, motor_flag_t *flag)
{
    /* 检测堵转 */
    if (flag->is_clogged && !motor->status.is_clogged) {
        motor->clog_detected_count++;
        printf("[ERROR] Motor 0x%02X: CLOGGED detected! Count=%lu\r\n", 
               motor->addr, (unsigned long)motor->clog_detected_count);
    }
    
    /* 更新状态 */
    motor->status = *flag;
}

/**
 * @brief       检查电机超时
 */
static void check_motor_timeout(motor_monitor_t *motor)
{
    uint32_t elapsed = HAL_GetTick() - motor->last_response_tick;
    
    /* 已上线的电机：检测是否掉线 */
    if (motor->online && elapsed > MOTOR_ONLINE_TIMEOUT_MS) {
        motor->online = false;
        motor->timeout_count++;
        motor->query_timeout_count++;
        printf("[MONITOR] Motor 0x%02X 掉线, last tick=%lu, timeout_count=%lu\r\n", motor->addr, (unsigned long)motor->last_response_tick, (unsigned long)motor->timeout_count);
    }

    /* 从未上线的电机：检测是否超时 */
    if (!motor->online && elapsed > MOTOR_ONLINE_TIMEOUT_MS) {
        motor->query_timeout_count++;
        motor->last_response_tick = HAL_GetTick();  /* 更新时间戳，继续下一轮检测 */
        if (motor->query_timeout_count == 1) {
            printf("[MONITOR] Motor 0x%02X 离线检测开始, tick=%lu\r\n", motor->addr, (unsigned long)motor->last_response_tick);
        }
    }

    /* 超时3次后确认为离线状态，停止查询（静默记录）*/
    if (!motor->online && motor->query_timeout_count >= 3 && !motor->status_determined) {
        motor->status_determined = true;
        printf("[MONITOR] Motor 0x%02X 已确认离线, 总超时次数=%lu\r\n", motor->addr, (unsigned long)motor->query_timeout_count);
    }
}
