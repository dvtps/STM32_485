/**
 ******************************************************************************
 * @file    usmart_interface.c
 * @author  STM32_485 Project
 * @version V3.1
 * @date    2025-12-01
 * @brief   USMART接口函数实现（桥接层�?
 ******************************************************************************
 */

#include "usmart_interface.h"
#include "app_config.h"  /* 检查FEATURE_MODBUS_ENABLE */

/* V3.6: 同步usart.c的宏定义 */
#ifndef ENABLE_INCREMENTAL_CRC
#define ENABLE_INCREMENTAL_CRC  0  /* DMA接收模式下禁用 */
#endif

#if FEATURE_MODBUS_ENABLE
/* 多电机管理器已归档（STM32F103容量不足）*/
#endif
#include "y_v2.h"      /* V3.0: Y系X固件协议驱动 */
#include "usart.h"  /* V3.5 Phase 8: CRC和FIFO统计 */
#include "printer_axis.h"  /* 3D打印机3轴控制 */
#include "motor_monitor.h"  /* V3.7: 电机监控系统 */
#include <stdio.h>
#include <stdlib.h>  /* for abs() */
#include <string.h>  /* memset */

/* ============ 单电机控制实现（保留有调试输出的函数）============ */

void motor_enable(uint8_t addr, uint8_t enable)
{
    Y_V2_En_Control(addr, enable ? true : false, false);
    printf("Motor#%d %s\r\n", addr, enable ? "ENABLED" : "DISABLED");
}

/* ============ 3D打印机3轴控制实现 ============ */

void printer_enable_all(void)
{
    printer_axis_enable_all();
}

void printer_disable_all(void)
{
    printer_axis_disable_all();
}

/* ===== 以脉冲数为单位的移动函数（底层调试用） ===== */

void printer_move_x(int32_t distance, uint16_t speed)
{
    printer_axis_move_relative(AXIS_X, distance, speed, 0);
    printf("X-axis move: %ld pulses\r\n", (long)distance);
}

void printer_move_y(int32_t distance, uint16_t speed)
{
    printer_axis_move_relative(AXIS_Y, distance, speed, 0);
    printf("Y-axis move: %ld pulses (dual motor sync)\r\n", (long)distance);
}

void printer_move_z(int32_t distance, uint16_t speed)
{
    printer_axis_move_relative(AXIS_Z, distance, speed, 0);
    printf("Z-axis move: %ld pulses\r\n", (long)distance);
}

void printer_move_xyz(int32_t x, int32_t y, int32_t z, uint16_t speed)
{
    printer_move_xyz_sync(x, y, z, speed);
}

/* ===== 以毫米为单位的移动函数（3D打印应用推荐） ===== */

void printer_move_x_mm(float distance_mm, uint16_t speed)
{
    printer_move_mm(AXIS_X, distance_mm, speed, 0);
    printf("X-axis: %.2f mm (%.0f pulses @ 160p/mm)\r\n", 
           distance_mm, distance_mm * PULSES_PER_MM);
}

/* ===== 整数版本mm移动函数（USMART兼容，精度0.1mm） ===== */

void printer_move_x_mm_int(int16_t distance_dmm, uint16_t speed)
{
    float distance_mm = fabsf((float)distance_dmm) / 10.0f;  /* 绝对值转毫米 */
    int32_t pulses = (int32_t)(distance_mm * PULSES_PER_MM);
    int8_t dir = (distance_dmm >= 0) ? 1 : -1;

    printf("[CMD] X轴移动: %c%.1fmm = %ld脉冲, 速度=%dRPM\r\n", 
           (dir > 0 ? '+' : '-'), distance_mm, (long)pulses * dir, speed);

    bool result = printer_move_mm(AXIS_X, distance_mm * dir, speed, 0);

    if (!result) {
        printf("[ERROR] 移动失败!\r\n");
    }
}

void printer_move_y_mm(float distance_mm, uint16_t speed)
{
    printer_move_mm(AXIS_Y, distance_mm, speed, 0);
    printf("Y-axis: %.2f mm (dual motor sync)\r\n", distance_mm);
}

void printer_move_z_mm(float distance_mm, uint16_t speed)
{
    printer_move_mm(AXIS_Z, distance_mm, speed, 0);
    printf("Z-axis: %.2f mm\r\n", distance_mm);
}

void printer_move_y_mm_int(int16_t distance_dmm, uint16_t speed)
{
    float distance_mm = fabsf((float)distance_dmm) / 10.0f;
    int32_t pulses = (int32_t)(distance_mm * PULSES_PER_MM);
    int8_t dir = (distance_dmm >= 0) ? 1 : -1;

    printf("[CMD] Y轴移动: %c%.1fmm = %ld脉冲 (双电机同步), 速度=%dRPM\r\n",
           (dir > 0 ? '+' : '-'), distance_mm, (long)pulses * dir, speed);

    bool result = printer_move_mm(AXIS_Y, distance_mm * dir, speed, 0);

    if (!result) {
        printf("[ERROR] 移动失败!\r\n");
    }
}

void printer_move_z_mm_int(int16_t distance_dmm, uint16_t speed)
{
    float distance_mm = fabsf((float)distance_dmm) / 10.0f;
    int32_t pulses = (int32_t)(distance_mm * PULSES_PER_MM);
    int8_t dir = (distance_dmm >= 0) ? 1 : -1;

    printf("[CMD] Z轴移动: %c%.1fmm = %ld脉冲, 速度=%dRPM\r\n",
           (dir > 0 ? '+' : '-'), distance_mm, (long)pulses * dir, speed);

    bool result = printer_move_mm(AXIS_Z, distance_mm * dir, speed, 0);

    if (!result) {
        printf("[ERROR] 移动失败!\r\n");
    }
}

void printer_xyz_mm(float x_mm, float y_mm, float z_mm, uint16_t speed)
{
    printer_move_xyz_mm(x_mm, y_mm, z_mm, speed);  /* 调用printer_axis.c中的实现 */
    printf("XYZ sync: X=%.2f Y=%.2f Z=%.2f mm\r\n", x_mm, y_mm, z_mm);
}

void printer_xyz_mm_int(int16_t x_dmm, int16_t y_dmm, int16_t z_dmm, uint16_t speed)
{
    float x_mm = x_dmm / 10.0f;
    float y_mm = y_dmm / 10.0f;
    float z_mm = z_dmm / 10.0f;
    
    printf("[CMD] XYZ同步移动: X=%d.%dmm Y=%d.%dmm Z=%d.%dmm, 速度=%dRPM\r\n",
           x_dmm/10, abs(x_dmm%10), y_dmm/10, abs(y_dmm%10), z_dmm/10, abs(z_dmm%10), speed);
    
    bool result = printer_move_xyz_mm(x_mm, y_mm, z_mm, speed);
    
    if (!result) {
        printf("[ERROR] 移动失败!\r\n");
    }
}

void printer_home_x(void)
{
    printer_axis_home(AXIS_X, 0);
}

void printer_home_y(void)
{
    printer_axis_home(AXIS_Y, 0);
}

void printer_home_z(void)
{
    printer_axis_home(AXIS_Z, 0);
}

void printer_home_all_axes(void)
{
    printer_home_all(0);
}

void printer_estop(void)
{
    printer_emergency_stop();
}

void printer_show_status(void)
{
    const printer_state_t *state = printer_get_state();
    printf("\r\n========== 3D Printer Status ==========\r\n");
    printf("All Homed: %s\r\n", state->all_homed ? "YES" : "NO");
    printf("E-Stop: %s\r\n", state->emergency_stop ? "ACTIVE" : "Clear");
    printf("Total Moves: %lu\r\n\r\n", (unsigned long)state->total_moves);
    
    const char *axis_names[] = {"X(Width)", "Y(Depth)", "Z(Height)"};
    for (int i = 0; i < 3; i++) {
        const axis_state_t *axis = &state->axes[i];
        printf("%s: En=%d Home=%d Pos=%.2fmm (%ldp) Speed=%d\r\n",
               axis_names[i],
               axis->enabled,
               axis->homed,
               axis->position_mm,
               (long)axis->position_pulses,
               axis->speed);
    }
    printf("========================================\r\n\r\n");
}

/* ============ V3.5 Phase 8 P1: 增量CRC调试实现 ============ */

/**
 * @brief       显示增量CRC统计信息
 * @note        V3.6注意：DMA接收模式下无增量CRC功能，此函数被禁用
 */
void crc_stats(void)
{
#if ENABLE_INCREMENTAL_CRC
    uint16_t current_crc = get_incremental_crc();
    uint16_t byte_count = get_crc_byte_count();
    uint32_t calc_count = get_crc_calc_count();
    
    printf("\r\n========== Incremental CRC Stats ==========\r\n");
    printf("Current CRC value:  0x%04X\r\n", current_crc);
    printf("Bytes calculated:   %u\r\n", byte_count);
    printf("Frames calculated:  %lu\r\n", (unsigned long)calc_count);
    printf("CRC16 Table:        %s\r\n", "Enabled (256 entries)");
    printf("Performance gain:   ~100 CPU cycles/frame\r\n");
    printf("============================================\r\n\r\n");
#else
    printf("\r\n[INFO] Incremental CRC disabled in DMA RX mode\r\n");
    printf("       (V3.6: DMA receives data directly, RXNE interrupt bypassed)\r\n\r\n");
#endif
}

/* ============ V3.6: TIM2实时定时器控制 ============ */
#if REALTIME_MOTOR_ENABLE

extern uint8_t tim2_rt_init(void);
extern void tim2_rt_start(void);
extern void tim2_rt_stop(void);
extern uint8_t tim2_rt_is_running(void);

/**
 * @brief       启动TIM2实时定时器
 */
void tim2_start(void)
{
    if (tim2_rt_is_running())
    {
        printf("[INFO] TIM2 is already running\r\n");
        return;
    }
    tim2_rt_start();
    printf("[TIM2] Started (10kHz, 100us period)\r\n");
}

/**
 * @brief       停止TIM2实时定时器
 */
void tim2_stop(void)
{
    if (!tim2_rt_is_running())
    {
        printf("[INFO] TIM2 is already stopped\r\n");
        return;
    }
    tim2_rt_stop();
    printf("[TIM2] Stopped\r\n");
}

/**
 * @brief       显示TIM2运行状态
 */
void tim2_status(void)
{
    printf("\r\n========== TIM2 Real-Time Timer Status ==========\r\n");
    printf("State:         %s\r\n", tim2_rt_is_running() ? "RUNNING" : "STOPPED");
    printf("Frequency:     10 kHz\r\n");
    printf("Period:        100 us\r\n");
    printf("Function:      rt_motor_tick_handler()\r\n");
    printf("Priority:      Preemption=1 (below DMA)\r\n");
    printf("=================================================\r\n\r\n");
}

#endif /* REALTIME_MOTOR_ENABLE */

/**
 * @brief       显示电机监控状态（V3.7反馈闭环）
 */
void motor_monitor_status(void)
{
    motor_monitor_print_status();
}

/**
 * @brief       显示FIFO统计信息
 * @note        监控FIFO溢出情况（Phase 8优化指标）
 */
void fifo_stats(void)
{
    uint32_t overflow_count = get_fifo_overflow_count();
    uint32_t idle_count = get_idle_interrupt_count();
    
    printf("\r\n========== FIFO Statistics ==========\r\n");
    printf("FIFO size:          256 bytes\r\n");
    printf("FIFO overflow:      %lu times\r\n", (unsigned long)overflow_count);
    printf("IDLE interrupts:    %lu times\r\n", (unsigned long)idle_count);
    
    if (idle_count > 0) {
        float overflow_rate = (float)overflow_count / idle_count * 100.0f;
        printf("Overflow rate:      %.2f%%\r\n", overflow_rate);
        
        if (overflow_rate < 2.0f) {
            printf("Status:             GOOD (target: <2%%)\r\n");
        } else if (overflow_rate < 5.0f) {
            printf("Status:             WARNING (2-5%%)\r\n");
        } else {
            printf("Status:             CRITICAL (>5%%)\r\n");
        }
    }
    printf("======================================\r\n\r\n");
}

/**
 * @brief       显示电机参数详细说明
 * @note        输入 motor_help() 查看电机参数含义
 */
void motor_help(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("       电机参数详细说明       \r\n");
    printf("========================================\r\n\r\n");
    
    printf("【基本参数】\r\n");
    printf("  addr   - 电机地址 (1-255)\r\n");
    printf("           X轴=0x01, Y轴左=0x02, Y轴右=0x03, Z轴=0x04\r\n\r\n");
    
    printf("  dir    - 旋转方向\r\n");
    printf("           0=顺时针(CW), 1=逆时针(CCW)\r\n\r\n");
    
    printf("  speed  - 转速 (0-5000 RPM)\r\n");
    printf("           推荐: 低速=100-300, 中速=300-800, 高速=800-2000\r\n\r\n");
    
    printf("  acc    - 加速度 (0-255)\r\n");
    printf("           0=立即启动, 1-255=梯形加减速\r\n");
    printf("           推荐: 低速=5-10, 中速=10-20, 高速=20-50\r\n\r\n");
    
    printf("  pulses - 脉冲数 (位置模式)\r\n");
    printf("           16细分: 3200脉冲 = 1圈旋转\r\n");
    printf("           常用: 1600=半圈, 3200=1圈, 6400=2圈\r\n\r\n");
    
    printf("【3D打印机机械参数】\r\n");
    printf("  丝杠导程   - 20mm/圈 (电机转1圈, 平台移动20mm)\r\n");
    printf("  脉冲密度   - 160脉冲/mm (3200脉冲/20mm)\r\n");
    printf("  理论分辨率 - 0.00625mm/脉冲 (20/3200)\r\n");
    printf("  实际精度   - 0.02mm (推荐最小移动单位)\r\n");
    printf("  行程范围   - X:300mm, Y:300mm, Z:400mm\r\n\r\n");
    
    printf("【毫米单位移动（推荐，0.1mm精度）】\r\n");
    printf("  注意: USMART不支持浮点数，使用分米单位(0.1mm)\r\n");
    printf("  示例: 10.5mm → 输入105, -20.0mm → 输入-200\r\n\r\n");
    
    printf("  1. X轴前进10.5mm\r\n");
    printf("     printer_move_x_mm_int(105,800)\r\n\r\n");
    
    printf("  2. Y轴后退20mm (负数表示反向)\r\n");
    printf("     printer_move_y_mm_int(-200,600)\r\n\r\n");
    
    printf("  3. Z轴上升5mm\r\n");
    printf("     printer_move_z_mm_int(50,200)\r\n\r\n");
    
    printf("  4. XYZ三轴同步移动（G0快速定位）\r\n");
    printf("     printer_xyz_mm_int(100,150,25,1000)\r\n");
    printf("     => X=10.0mm, Y=15.0mm, Z=2.5mm\r\n\r\n");
    
    printf("【Y_V2底层API直接调用（V3.0 X固件协议）】\r\n");
    printf("  注意: Y_V2使用角度(float)和加速度(RPM/S)\r\n");
    printf("  raf=0相对上次, raf=1绝对, raf=2相对当前\r\n");
    printf("  snF=0立即执行, snF=1同步等待\r\n\r\n");
    
    printf("  1. 位置控制 - 电机1顺时针转1圈 (300RPM, 360度)\r\n");
    printf("     Y_V2_Bypass_Pos_Control(1,0,300.0,360.0,0,0)\r\n\r\n");
    
    printf("  2. 速度控制 - 电机1持续转动 (500RPM, 加速度1000RPM/S)\r\n");
    printf("     Y_V2_Vel_Control(1,0,1000,500.0,0)\r\n\r\n");
    
    printf("  3. 立即停止 - 急停电机1\r\n");
    printf("     Y_V2_Stop_Now(1,0)\r\n\r\n");
    
    printf("  4. 回零 - 电机1模式0回零\r\n");
    printf("     Y_V2_Origin_Trigger_Return(1,0,0)\r\n\r\n");
    
    printf("  5. 查询转速 - 读取电机1实时转速(参数14)\r\n");
    printf("     Y_V2_Read_Sys_Params(1,14)\r\n\r\n");
    
    printf("【脉冲单位移动（3轴封装，推荐）】\r\n");
    printf("  1. X轴前进20mm (3200脉冲 = 20mm)\r\n");
    printf("     printer_move_x(3200,800)\r\n\r\n");
    
    printf("【系统功能】\r\n");
    printf("  • 全轴回零: printer_home_all_axes()  // 自动Z→Y→X\r\n");
    printf("  • 紧急停止: printer_estop()\r\n");
    printf("  • 查询状态: printer_show_status()\r\n\r\n");
    
    printf("【注意事项】\r\n");
    printf("  • 首次使用前必须使能: motor_enable(addr,1)\r\n");
    printf("  • 速度过高可能导致失步，建议<1000 RPM\r\n");
    printf("  • 加速度过小会导致启动缓慢\r\n");
    printf("  • Y轴双电机需同步: printer_move_y()自动处理\r\n");
    printf("  • 紧急情况使用: printer_estop()\r\n\r\n");
    
    printf("========================================\r\n");
    printf("输入 '?' 查看所有可用命令\r\n");
    printf("========================================\r\n\r\n");
}
