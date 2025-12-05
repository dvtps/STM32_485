/**
 ****************************************************************************************************
 * @file        printer_axis.c
 * @author      STM32_485 Project
 * @version     V1.0
 * @date        2025-12-03
 * @brief       3D打印机3轴（4电机）控制实现
 ****************************************************************************************************
 */

#include "printer_axis.h"
#include "y_v2.h"       /* V3.0: Y系列X固件协议驱动 */
#include "error_handler.h"
#include "logger.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  /* for abs() */

/* ============================================================================
 * 私有变量
 * ============================================================================ */

/* 打印机状态 */
static printer_state_t g_printer = {0};

/* 轴配置表（电机地址、默认速度、默认加速度） */
typedef struct {
    uint8_t motor_addrs[2];  /* 电机地址（最多2个，Y轴双电机） */
    uint8_t motor_count;     /* 电机数量 */
    uint16_t default_speed;  /* 默认速度 */
    uint8_t default_acc;     /* 默认加速度 */
} axis_config_t;

static const axis_config_t g_axis_config[AXIS_COUNT] = {
    /* X轴（宽度）*/
    { {AXIS_X_ADDR, 0}, 1, AXIS_X_DEFAULT_SPEED, AXIS_X_DEFAULT_ACC },
    /* Y轴（深度，双电机）*/
    { {AXIS_Y_ADDR_LEFT, AXIS_Y_ADDR_RIGHT}, 2, AXIS_Y_DEFAULT_SPEED, AXIS_Y_DEFAULT_ACC },
    /* Z轴（高度）*/
    { {AXIS_Z_ADDR, 0}, 1, AXIS_Z_DEFAULT_SPEED, AXIS_Z_DEFAULT_ACC }
};

/* ============================================================================
 * 私有函数
 * ============================================================================ */

/**
 * @brief       毫米转换为脉冲数
 * @param       mm: 距离（毫米）
 * @return      脉冲数
 * @note        基于丝杠导程20mm/圈，3200脉冲/圈，计算得160脉冲/mm
 */
static uint32_t mm_to_pulses(float mm)
{
    return (uint32_t)roundf(mm * PULSES_PER_MM);
}

/**
 * @brief       脉冲数转换为毫米
 * @param       pulses: 脉冲数
 * @return      距离（毫米）
 */
static float pulses_to_mm(uint32_t pulses)
{
    return (float)pulses * MM_PER_PULSE;
}

/**
 * @brief       检查轴行程是否超限（相对移动）
 * @param       axis: 轴枚举
 * @param       distance_mm: 相对移动距离（毫米）
 * @return      true=在范围内, false=超限
 */
static bool check_axis_limit(printer_axis_t axis, float distance_mm)
{
    /* 计算目标位置 */
    float current_pos = g_printer.axes[axis].position_mm;
    float target_pos = current_pos + distance_mm;
    
    /* 检查目标位置是否在允许范围内 */
    float min_pos = 0, max_pos = 0;
    switch (axis) {
        case AXIS_X: 
            min_pos = AXIS_X_MIN_POS_MM; 
            max_pos = AXIS_X_MAX_POS_MM; 
            break;
        case AXIS_Y: 
            min_pos = AXIS_Y_MIN_POS_MM; 
            max_pos = AXIS_Y_MAX_POS_MM; 
            break;
        case AXIS_Z: 
            min_pos = AXIS_Z_MIN_POS_MM; 
            max_pos = AXIS_Z_MAX_POS_MM; 
            break;
        default: 
            return false;
    }
    
    return (target_pos >= min_pos && target_pos <= max_pos);
}

/**
 * @brief       获取轴默认速度
 */
static uint16_t get_axis_speed(printer_axis_t axis, uint16_t speed)
{
    return (speed == 0) ? g_axis_config[axis].default_speed : speed;
}

/**
 * @brief       获取轴默认加速度
 */
static uint8_t get_axis_acc(printer_axis_t axis, uint8_t acc)
{
    return (acc == 0) ? g_axis_config[axis].default_acc : acc;
}

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

/**
 * @brief       初始化3D打印机轴控制系统
 */
void printer_axis_init(void)
{
    /* 清零状态 */
    memset(&g_printer, 0, sizeof(printer_state_t));
    
#ifndef NDEBUG
    printf("\r\n");
    LOG_PRINTER("3轴 4电机系统初始化");
    LOG_PRINTER("机械参数: 导程%.1fmm/圈, 理论分辨率%.5fmm, 装配精度±%.2fmm", 
                LEAD_SCREW_PITCH_MM, THEORETICAL_RESOLUTION_MM, MECHANICAL_ACCURACY_MM);
    LOG_PRINTER("脉冲密度: %d脉冲/mm", (int)PULSES_PER_MM);
    LOG_PRINTER("X轴: 电机 0x%02X (宽度, 行程%.0fmm)", AXIS_X_ADDR, AXIS_X_MAX_TRAVEL_MM);
    LOG_PRINTER("Y轴: 电机 0x%02X + 0x%02X (深度, 同步, 行程%.0fmm)", 
                AXIS_Y_ADDR_LEFT, AXIS_Y_ADDR_RIGHT, AXIS_Y_MAX_TRAVEL_MM);
    LOG_PRINTER("Z轴: 电机 0x%02X (高度, 行程%.0fmm)", AXIS_Z_ADDR, AXIS_Z_MAX_TRAVEL_MM);
    
#if PRINTER_ENABLE_ALL_ON_START
    LOG_PRINTER("正在使能所有电机...");
#endif

#if PRINTER_HOME_ON_STARTUP
    LOG_PRINTER("正在执行全轴回零...");
#endif
#endif

#if PRINTER_ENABLE_ALL_ON_START
    printer_axis_enable_all();
    HAL_Delay(100);
#endif

#if PRINTER_HOME_ON_STARTUP
    printer_home_all(MOTOR_HOME_MODE);
#endif

#ifndef NDEBUG
    LOG_PRINTER("初始化完成");
#endif
}

/**
 * @brief       使能指定轴
 */
bool printer_axis_enable(printer_axis_t axis)
{
    CHECK_PARAM(axis < AXIS_COUNT);
    
    const axis_config_t *cfg = &g_axis_config[axis];
    
    /* 使能该轴的所有电机 */
    for (uint8_t i = 0; i < cfg->motor_count; i++) {
        Y_V2_En_Control(cfg->motor_addrs[i], true, false);
        HAL_Delay(10);
    }
    
    g_printer.axes[axis].enabled = true;
    return true;
}

/**
 * @brief       失能指定轴
 */
bool printer_axis_disable(printer_axis_t axis)
{
    CHECK_PARAM(axis < AXIS_COUNT);
    
    const axis_config_t *cfg = &g_axis_config[axis];
    
    /* 失能该轴的所有电机 */
    for (uint8_t i = 0; i < cfg->motor_count; i++) {
        Y_V2_En_Control(cfg->motor_addrs[i], false, false);
        HAL_Delay(10);
    }
    
    g_printer.axes[axis].enabled = false;
    return true;
}

/**
 * @brief       使能所有轴（广播命令）
 */
bool printer_axis_enable_all(void)
{
    Y_V2_En_Control(0, true, false);  /* 广播地址0 */
    
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_printer.axes[i].enabled = true;
    }
    
    LOG_PRINTER("所有轴已使能");
    return true;
}

/**
 * @brief       失能所有轴（广播命令）
 */
bool printer_axis_disable_all(void)
{
    Y_V2_En_Control(0, false, false);  /* 广播地址0 */
    
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_printer.axes[i].enabled = false;
    }
    
    printf("[Printer] All axes disabled\r\n");
    return true;
}

/**
 * @brief       单轴相对运动
 */
bool printer_axis_move_relative(printer_axis_t axis, int32_t distance, uint16_t speed, uint8_t acc)
{
    CHECK_PARAM(axis < AXIS_COUNT);
    
    if (!g_printer.axes[axis].enabled) {
        printf("[Printer] Error: Axis %d not enabled\r\n", axis);
        return false;
    }

    if (g_printer.emergency_stop) {
        printf("[Printer] Error: Emergency stop active\r\n");
        return false;
    }

    /* 解析方向和距离 */
    uint8_t dir = (distance >= 0) ? 0 : 1;
    uint32_t pulses = (distance >= 0) ? distance : -distance;
    float angle = PULSES_TO_DEGREES(pulses);  /* 转换为角度（Y_V2协议） */

    /* 获取速度和加速度 */
    uint16_t actual_speed = get_axis_speed(axis, speed);
    uint8_t actual_acc = get_axis_acc(axis, acc);

    const axis_config_t *cfg = &g_axis_config[axis];

    printf("[DEBUG] axis_move_relative: axis=%d, addr[0]=0x%02X, dir=%d, pulses=%lu, angle=%d.%02d, speed=%d, acc=%d, motor_count=%d\r\n",
        axis, cfg->motor_addrs[0], dir, (unsigned long)pulses, (int)angle, ((int)(angle * 100) % 100), actual_speed, actual_acc, cfg->motor_count);

    /* 如果是Y轴双电机，需要同步运动 */
    if (cfg->motor_count == 2 && PRINTER_SYNC_Y_AXIS) {
        printf("[DEBUG] Y轴双电机同步: addrL=0x%02X, addrR=0x%02X\r\n", cfg->motor_addrs[0], cfg->motor_addrs[1]);
        Y_V2_Bypass_Pos_Control(cfg->motor_addrs[0], dir, (float)actual_speed, angle,
                                2, true);
        HAL_Delay(10);
        Y_V2_Bypass_Pos_Control(cfg->motor_addrs[1], dir, (float)actual_speed, angle,
                                2, true);
        HAL_Delay(10);

        /* 触发同步运动 */
        Y_V2_Synchronous_Motion(0);
    } else {
        /* 单电机轴，直接运动 */
        for (uint8_t i = 0; i < cfg->motor_count; i++) {
            printf("[DEBUG] 单电机运动: addr=0x%02X, dir=%d, speed=%d, angle=%d.%02d\r\n", cfg->motor_addrs[i], dir, actual_speed, (int)angle, ((int)(angle * 100) % 100));
            Y_V2_Bypass_Pos_Control(cfg->motor_addrs[i], dir, (float)actual_speed, angle,
                                    2, false);
            HAL_Delay(10);
        }
    }

    /* 更新状态 */
    g_printer.axes[axis].position_pulses += distance;
    g_printer.axes[axis].position_mm = pulses_to_mm((uint32_t)abs(g_printer.axes[axis].position_pulses));
    g_printer.axes[axis].speed = actual_speed;
    g_printer.axes[axis].acceleration = actual_acc;
    g_printer.axes[axis].last_move_tick = HAL_GetTick();
    g_printer.total_moves++;

    return true;
}

/**
 * @brief       单轴绝对运动
 */
bool printer_axis_move_absolute(printer_axis_t axis, int32_t target_pos, uint16_t speed, uint8_t acc)
{
    CHECK_PARAM(axis < AXIS_COUNT);
    
    if (!g_printer.axes[axis].homed) {
        printf("[Printer] Warning: Axis %d not homed\r\n", axis);
    }
    
    int32_t distance = target_pos - g_printer.axes[axis].position_pulses;
    return printer_axis_move_relative(axis, distance, speed, acc);
}

/**
 * @brief       XYZ三轴同步运动
 */
bool printer_move_xyz_sync(int32_t x_dist, int32_t y_dist, int32_t z_dist, uint16_t speed)
{
    if (g_printer.emergency_stop) {
        printf("[Printer] Error: Emergency stop active\r\n");
        return false;
    }
    
    uint8_t cmd_count = 0;
    
    /* X轴 */
    if (x_dist != 0) {
        uint8_t dir = (x_dist >= 0) ? 0 : 1;
        uint32_t pulses = (x_dist >= 0) ? x_dist : -x_dist;
        float angle = PULSES_TO_DEGREES(pulses);
        uint16_t spd = speed ? speed : AXIS_X_DEFAULT_SPEED;
        Y_V2_Bypass_Pos_Control(AXIS_X_ADDR, dir, (float)spd, angle, 
                                2, true);
        HAL_Delay(10);
        cmd_count++;
    }
    
    /* Y轴（双电机同步）*/
    if (y_dist != 0) {
        uint8_t dir = (y_dist >= 0) ? 0 : 1;
        uint32_t pulses = (y_dist >= 0) ? y_dist : -y_dist;
        float angle = PULSES_TO_DEGREES(pulses);
        uint16_t spd = speed ? speed : AXIS_Y_DEFAULT_SPEED;
        Y_V2_Bypass_Pos_Control(AXIS_Y_ADDR_LEFT, dir, (float)spd, angle, 
                                2, true);
        HAL_Delay(10);
        Y_V2_Bypass_Pos_Control(AXIS_Y_ADDR_RIGHT, dir, (float)spd, angle, 
                                2, true);
        HAL_Delay(10);
        cmd_count += 2;
    }
    
    /* Z轴 */
    if (z_dist != 0) {
        uint8_t dir = (z_dist >= 0) ? 0 : 1;
        uint32_t pulses = (z_dist >= 0) ? z_dist : -z_dist;
        float angle = PULSES_TO_DEGREES(pulses);
        uint16_t spd = speed ? speed : AXIS_Z_DEFAULT_SPEED;
        Y_V2_Bypass_Pos_Control(AXIS_Z_ADDR, dir, (float)spd, angle, 
                                2, true);
        HAL_Delay(10);
        cmd_count++;
    }
    
    /* 触发同步运动 */
    if (cmd_count > 0) {
        Y_V2_Synchronous_Motion(0);
        
        /* 更新位置 */
        g_printer.axes[AXIS_X].position_pulses += x_dist;
        g_printer.axes[AXIS_X].position_mm = pulses_to_mm((uint32_t)abs(g_printer.axes[AXIS_X].position_pulses));
        g_printer.axes[AXIS_Y].position_pulses += y_dist;
        g_printer.axes[AXIS_Y].position_mm = pulses_to_mm((uint32_t)abs(g_printer.axes[AXIS_Y].position_pulses));
        g_printer.axes[AXIS_Z].position_pulses += z_dist;
        g_printer.axes[AXIS_Z].position_mm = pulses_to_mm((uint32_t)abs(g_printer.axes[AXIS_Z].position_pulses));
        g_printer.total_moves++;
        
        printf("[Printer] XYZ sync move: X=%ld Y=%ld Z=%ld\r\n", 
               (long)x_dist, (long)y_dist, (long)z_dist);
        return true;
    }
    
    return false;
}

/**
 * @brief       XY平面运动
 */
bool printer_move_xy(int32_t x_dist, int32_t y_dist, uint16_t speed)
{
    return printer_move_xyz_sync(x_dist, y_dist, 0, speed);
}

/**
 * @brief       指定轴回零
 */
bool printer_axis_home(printer_axis_t axis, uint8_t mode)
{
    CHECK_PARAM(axis < AXIS_COUNT);
    CHECK_PARAM(mode <= 3);
    
    printf("[Printer] Homing axis %d (mode %d)...\r\n", axis, mode);
    
    const axis_config_t *cfg = &g_axis_config[axis];
    
    /* 所有电机回零 */
    for (uint8_t i = 0; i < cfg->motor_count; i++) {
        Y_V2_Origin_Trigger_Return(cfg->motor_addrs[i], mode, false);
        HAL_Delay(50);
    }
    
    /* 标记已回零，位置重置为原点 */
    g_printer.axes[axis].homed = true;
    g_printer.axes[axis].position_mm = 0.0f;
    g_printer.axes[axis].position_pulses = 0;
    
    return true;
}

/**
 * @brief       所有轴依次回零（Z→Y→X顺序）
 */
bool printer_home_all(uint8_t mode)
{
    printf("[Printer] Homing all axes (Z→Y→X)...\r\n");
    
    /* 先回Z轴（抬起，避免碰撞）*/
    printer_axis_home(AXIS_Z, mode);
    HAL_Delay(3000);  /* 等待Z轴回零完成 */
    
    /* 回Y轴 */
    printer_axis_home(AXIS_Y, mode);
    HAL_Delay(3000);
    
    /* 回X轴 */
    printer_axis_home(AXIS_X, mode);
    HAL_Delay(3000);
    
    g_printer.all_homed = true;
    printf("[Printer] All axes homed\r\n");
    
    return true;
}

/**
 * @brief       紧急停止所有轴
 */
bool printer_emergency_stop(void)
{
    printf("[Printer] *** EMERGENCY STOP ***\r\n");
    
    /* 广播紧急停止 */
    Y_V2_Stop_Now(0, false);
    
    g_printer.emergency_stop = true;
    
    return true;
}

/**
 * @brief       获取轴状态
 */
const axis_state_t* printer_get_axis_state(printer_axis_t axis)
{
    if (axis >= AXIS_COUNT) return NULL;
    return &g_printer.axes[axis];
}

/**
 * @brief       获取打印机状态
 */
const printer_state_t* printer_get_state(void)
{
    return &g_printer;
}

/* ============================================================================
 * 以毫米为单位的运动API实现
 * ============================================================================ */

/**
 * @brief       单轴移动（毫米单位）
 */
bool printer_move_mm(printer_axis_t axis, float distance_mm, uint16_t speed, uint8_t acc)
{
    CHECK_PARAM(axis < AXIS_COUNT);
    
    /* 检查电机是否使能 */
    if (!g_printer.axes[axis].enabled) {
        printf("[ERROR] Axis %c not enabled! Call printer_enable_all() first.\r\n", 'X'+axis);
        return false;
    }
    
    /* 检查行程限制 */
    if (!check_axis_limit(axis, distance_mm)) {
        LOG_PRINTER("错误: %c轴移动距离%.2fmm超出行程限制", 'X'+axis, distance_mm);
        return false;
    }
    
        /* 方向与绝对值分离，始终正脉冲+方向 */
        int8_t dir = (distance_mm >= 0) ? 1 : -1;
        float abs_mm = fabsf(distance_mm);
        uint32_t pulses = mm_to_pulses(abs_mm);
        int32_t signed_pulses = pulses * dir;

        printf("[DEBUG] Move: %c%d.%02dmm = %ld pulses, speed=%d\r\n",
            (dir > 0 ? '+' : '-'), (int)abs_mm, ((int)(abs_mm * 100) % 100), (long)signed_pulses, (int)speed);

        /* 直接传递方向到下层 */
        return printer_axis_move_relative(axis, signed_pulses, speed, acc);
}

/**
 * @brief       XYZ三轴同步移动（毫米单位）
 */
bool printer_move_xyz_mm(float x_mm, float y_mm, float z_mm, uint16_t speed)
{
    /* 检查行程限制 */
    if (!check_axis_limit(AXIS_X, x_mm)) {
        LOG_PRINTER("错误: X轴移动%.2fmm超出行程限制", x_mm);
        return false;
    }
    if (!check_axis_limit(AXIS_Y, y_mm)) {
        LOG_PRINTER("错误: Y轴移动%.2fmm超出行程限制", y_mm);
        return false;
    }
    if (!check_axis_limit(AXIS_Z, z_mm)) {
        LOG_PRINTER("错误: Z轴移动%.2fmm超出行程限制", z_mm);
        return false;
    }
    
    /* 转换mm到脉冲数 */
    int32_t x_pulses = (x_mm >= 0) ? (int32_t)mm_to_pulses(x_mm) : -(int32_t)mm_to_pulses(-x_mm);
    int32_t y_pulses = (y_mm >= 0) ? (int32_t)mm_to_pulses(y_mm) : -(int32_t)mm_to_pulses(-y_mm);
    int32_t z_pulses = (z_mm >= 0) ? (int32_t)mm_to_pulses(z_mm) : -(int32_t)mm_to_pulses(-z_mm);
    
    /* 调用原有的脉冲数API */
    return printer_move_xyz_sync(x_pulses, y_pulses, z_pulses, speed);
}

/**
 * @brief       XY平面移动（毫米单位）
 */
bool printer_move_xy_mm(float x_mm, float y_mm, uint16_t speed)
{
    return printer_move_xyz_mm(x_mm, y_mm, 0, speed);
}

/**
 * @brief       获取轴当前位置（毫米单位）
 */
float printer_get_position_mm(printer_axis_t axis)
{
    if (axis >= AXIS_COUNT) return 0.0f;
    return g_printer.axes[axis].position_mm;
}

/**
 * @brief       设置轴位置（手动校准用）
 */
void printer_set_axis_position(printer_axis_t axis, int32_t position)
{
    if (axis < AXIS_COUNT) {
        g_printer.axes[axis].position_pulses = position;
        g_printer.axes[axis].position_mm = pulses_to_mm((uint32_t)abs(position));
    }
}

/**
 * @brief       清除急停状态
 */
void printer_clear_emergency_stop(void)
{
    g_printer.emergency_stop = false;
    printf("[Printer] Emergency stop cleared\r\n");
}
