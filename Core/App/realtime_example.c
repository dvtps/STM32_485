/**
 ****************************************************************************************************
 * @file        realtime_example.c
 * @author      STM32_485 Project
 * @version     V1.0
 * @date        2025-12-03
 * @brief       微秒级实时电机控制集成示例
 ****************************************************************************************************
 * @attention   使用前需在app_config.h中设置：
 *              #define REALTIME_MOTOR_ENABLE 1
 *              
 *              然后在main.c中包含此文件：
 *              #include "realtime_example.c"
 ****************************************************************************************************
 */


#include "realtime_motor.h"
#include "printer_axis.h"
#include "app_config.h"
#include <stdio.h>
#include "stm32f1xx_hal.h"  // 修复 HAL_Delay/HAL_GetTick 未声明

#if REALTIME_MOTOR_ENABLE

/* ====================================================================================
   示例1：基本命令测试（<10μs延迟）
   ==================================================================================== */

void rt_example_basic_commands(void)
{
    printf("\r\n===== 示例1：基本命令测试 =====\r\n");
    
    /* 使能电机（<5μs） */
    RT_MOTOR_ENABLE(0x01);
    printf("X轴电机已使能\r\n");
    HAL_Delay(100);  /* 等待电机响应 */
    
    /* 位置运动：转1圈（<8μs） */
    RT_MOTOR_POS(0x01, 0, 800, 20, 3200);
    printf("X轴开始转动（1圈，800RPM）\r\n");
    HAL_Delay(5000);  /* 等待运动完成 */
    
    /* 速度运动：连续转动（<8μs） */
    RT_MOTOR_VEL(0x01, 0, 500, 20);
    printf("X轴连续转动（500RPM）\r\n");
    HAL_Delay(3000);
    
    /* 急停（<5μs） */
    RT_MOTOR_STOP(0x01);
    printf("X轴急停\r\n");
    HAL_Delay(100);
    
    /* 失能电机（<5μs） */
    RT_MOTOR_DISABLE(0x01);
    printf("X轴电机已失能\r\n");
    
    printf("示例1完成！\r\n\r\n");
}

/* ====================================================================================
   示例2：Y轴双电机同步（<15μs延迟）
   ==================================================================================== */

void rt_example_y_axis_sync(void)
{
    printf("\r\n===== 示例2：Y轴双电机同步 =====\r\n");
    
    /* 使能Y轴双电机 */
    RT_MOTOR_ENABLE(0x02);
    RT_MOTOR_ENABLE(0x03);
    printf("Y轴双电机已使能\r\n");
    HAL_Delay(100);
    
    /* 方法1：使用批量同步接口（推荐） */
    uint8_t y_motors[] = {0x02, 0x03};
    uint32_t params[] = {0, 600, 15, 1600};  /* dir, speed, acc, pulses */
    rt_motor_sync_submit(y_motors, 2, RT_CMD_POS_MOVE, params);
    printf("Y轴双电机同步运动（半圈，600RPM）\r\n");
    HAL_Delay(3000);
    
    /* 方法2：手动提交+同步触发 */
    RT_MOTOR_POS(0x02, 1, 600, 15, 1600);  /* 反向运动 */
    RT_MOTOR_POS(0x03, 1, 600, 15, 1600);
    RT_MOTOR_SYNC();  /* 同步触发 */
    printf("Y轴双电机同步返回\r\n");
    HAL_Delay(3000);
    
    /* 失能 */
    RT_MOTOR_DISABLE(0x02);
    RT_MOTOR_DISABLE(0x03);
    printf("Y轴双电机已失能\r\n");
    
    printf("示例2完成！\r\n\r\n");
}

/* ====================================================================================
   示例3：XYZ三轴协同运动（<30μs延迟）
   ==================================================================================== */

void rt_example_xyz_coordinated(void)
{
    printf("\r\n===== 示例3：XYZ三轴协同运动 =====\r\n");
    
    /* 使能所有4台电机 */
    RT_MOTOR_ENABLE(0x01);  /* X */
    RT_MOTOR_ENABLE(0x02);  /* Y左 */
    RT_MOTOR_ENABLE(0x03);  /* Y右 */
    RT_MOTOR_ENABLE(0x04);  /* Z */
    printf("XYZ三轴（4台电机）已使能\r\n");
    HAL_Delay(200);
    
    /* 同时提交3轴命令，最后同步触发 */
    RT_MOTOR_POS(0x01, 0, 800, 20, 3200);   /* X轴：1圈 */
    RT_MOTOR_POS(0x02, 0, 600, 15, 1600);   /* Y左：半圈 */
    RT_MOTOR_POS(0x03, 0, 600, 15, 1600);   /* Y右：半圈 */
    RT_MOTOR_POS(0x04, 1, 400, 10, 800);    /* Z轴：1/4圈 */
    RT_MOTOR_SYNC();  /* 4轴同步启动 */
    
    printf("XYZ三轴协同运动中...\r\n");
    HAL_Delay(8000);  /* 等待最慢的轴完成 */
    
    /* 失能所有电机 */
    RT_MOTOR_DISABLE(0x01);
    RT_MOTOR_DISABLE(0x02);
    RT_MOTOR_DISABLE(0x03);
    RT_MOTOR_DISABLE(0x04);
    printf("XYZ三轴已失能\r\n");
    
    printf("示例3完成！\r\n\r\n");
}

/* ====================================================================================
   示例4：高速连续运动（测试吞吐量）
   ==================================================================================== */

void rt_example_high_throughput(void)
{
    printf("\r\n===== 示例4：高速连续运动 =====\r\n");
    
    RT_MOTOR_ENABLE(0x01);
    printf("X轴电机已使能，开始高速连续运动测试...\r\n");
    HAL_Delay(100);
    
    uint32_t start_tick = HAL_GetTick();
    uint16_t cmd_count = 0;
    
    /* 连续提交100个位置命令（测试队列处理能力） */
    for (int i = 0; i < 100; i++)
    {
        /* 检查队列可用空间 */
        while (rt_motor_queue_available() < 2) {
            /* 等待队列有空间（实际中可以做其他任务）*/
        }
        
        /* 提交命令：每次转1/10圈 */
        RT_MOTOR_POS(0x01, i % 2, 800, 20, 320);  /* 交替方向 */
        cmd_count++;
    }
    
    /* 等待所有命令发送完成 */
    rt_motor_flush();
    uint32_t elapsed_ms = HAL_GetTick() - start_tick;
    
    printf("发送%d个命令耗时: %u ms\r\n", cmd_count, elapsed_ms);
    printf("平均吞吐量: %lu cmd/s\r\n", (cmd_count * 1000UL) / elapsed_ms);
    
    HAL_Delay(10000);  /* 等待运动完成 */
    RT_MOTOR_DISABLE(0x01);
    
    printf("示例4完成！\r\n\r\n");
}

/* ====================================================================================
   示例5：性能监控与统计
   ==================================================================================== */

#if RT_ENABLE_PROFILING

void rt_example_performance_stats(void)
{
    printf("\r\n===== 示例5：性能监控 =====\r\n");
    
    /* 重置统计 */
    rt_motor_reset_stats();
    
    RT_MOTOR_ENABLE(0x01);
    HAL_Delay(100);
    
    /* 发送50个命令 */
    for (int i = 0; i < 50; i++)
    {
        RT_MOTOR_POS(0x01, 0, 800, 20, 640);  /* 1/5圈 */
        while (rt_motor_queue_available() == 0);
    }
    
    rt_motor_flush();
    
    /* 获取统计信息 */
    rt_perf_stats_t stats;
    rt_motor_get_stats(&stats);
    
    printf("\r\n===== 性能统计报告 =====\r\n");
    printf("命令提交: %u\r\n", stats.cmd_submit_count);
    printf("命令完成: %u\r\n", stats.cmd_complete_count);
    printf("DMA启动次数: %u\r\n", stats.dma_start_count);
    printf("队列满次数: %u\r\n", stats.queue_full_count);
    printf("最大延迟: %uμs\r\n", stats.max_latency_us);
    printf("最小延迟: %uμs\r\n", stats.min_latency_us);
    printf("平均延迟: %uμs\r\n", stats.avg_latency_us);
    
    /* 测量单次命令延迟 */
    uint32_t latency = rt_motor_measure_latency(0x01, RT_CMD_STOP);
    printf("单次急停命令延迟: %uμs\r\n", latency);
    
    RT_MOTOR_DISABLE(0x01);
    printf("\r\n示例5完成！\r\n\r\n");
}

#endif

/* ====================================================================================
   示例6：模拟G代码实时执行（高速3D打印）
   ==================================================================================== */

void rt_example_gcode_simulation(void)
{
    printf("\r\n===== 示例6：G代码实时执行 =====\r\n");
    
    /* 使能所有轴 */
    RT_MOTOR_ENABLE(0x01);
    RT_MOTOR_ENABLE(0x02);
    RT_MOTOR_ENABLE(0x03);
    RT_MOTOR_ENABLE(0x04);
    HAL_Delay(200);
    
    printf("模拟G代码执行：\r\n");
    
    /* G28 - 回零（使用原有接口，实时模式不影响） */
    printf("G28 ; Home all axes\r\n");
    /* 注意：回零命令仍使用Emm_V5_Origin_*，因为需要等待完成 */
    
    /* G1 X100 Y50 Z2 F12000 - 快速移动 */
    printf("G1 X100 Y50 Z2 F12000 ; Move to position\r\n");
    int32_t x_steps = 100 * 32;  /* 假设100mm = 3200步 */
    int32_t y_steps = 50 * 32;
    int32_t z_steps = 2 * 32;
    
    RT_MOTOR_POS(0x01, 0, 1200, 30, x_steps);
    RT_MOTOR_POS(0x02, 0, 1200, 30, y_steps);
    RT_MOTOR_POS(0x03, 0, 1200, 30, y_steps);
    RT_MOTOR_POS(0x04, 0, 400, 10, z_steps);
    RT_MOTOR_SYNC();
    HAL_Delay(5000);
    
    /* G1 X50 Y100 F6000 - 中速移动 */
    printf("G1 X50 Y100 F6000 ; Next position\r\n");
    x_steps = 50 * 32;
    y_steps = 100 * 32;
    
    RT_MOTOR_POS(0x01, 1, 600, 20, x_steps);  /* 反向 */
    RT_MOTOR_POS(0x02, 0, 600, 20, y_steps);
    RT_MOTOR_POS(0x03, 0, 600, 20, y_steps);
    RT_MOTOR_SYNC();
    HAL_Delay(5000);
    
    /* M84 - 失能所有电机 */
    printf("M84 ; Disable motors\r\n");
    RT_MOTOR_DISABLE(0x01);
    RT_MOTOR_DISABLE(0x02);
    RT_MOTOR_DISABLE(0x03);
    RT_MOTOR_DISABLE(0x04);
    
    printf("示例6完成！\r\n\r\n");
}

/* ====================================================================================
   主测试函数（在main.c中调用）
   ==================================================================================== */

void realtime_motor_examples_run(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  微秒级实时电机控制演示\r\n");
    printf("  STM32F103C8 @ 72MHz\r\n");
    printf("  目标延迟: <10μs\r\n");
    printf("========================================\r\n\r\n");
    
    /* 依次运行所有示例 */
    rt_example_basic_commands();      /* 示例1：基本命令 */
    HAL_Delay(1000);
    
    rt_example_y_axis_sync();         /* 示例2：Y轴同步 */
    HAL_Delay(1000);
    
    rt_example_xyz_coordinated();     /* 示例3：三轴协同 */
    HAL_Delay(1000);
    
    rt_example_high_throughput();     /* 示例4：高吞吐量 */
    HAL_Delay(1000);
    
#if RT_ENABLE_PROFILING
    rt_example_performance_stats();   /* 示例5：性能统计 */
    HAL_Delay(1000);
#endif
    
    rt_example_gcode_simulation();    /* 示例6：G代码模拟 */
    
    printf("\r\n所有示例运行完成！\r\n");
    printf("========================================\r\n\r\n");
}

#else

/* 如果未启用实时模式，提供空实现 */
void realtime_motor_examples_run(void)
{
    printf("实时电机控制未启用，请在app_config.h中设置：\r\n");
    printf("#define REALTIME_MOTOR_ENABLE 1\r\n");
}

#endif /* REALTIME_MOTOR_ENABLE */
