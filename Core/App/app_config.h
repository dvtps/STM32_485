/**
 ****************************************************************************************************
 * @file        app_config.h
 * @author      STM32_485项目
 * @version     V3.6 (3D Printer Edition)
 * @date        2025-12-03
 * @brief       3D打印机3轴4电机控制系统配置
 ****************************************************************************************************
 * @attention   硬件配置：
 *              - X轴(宽度): 1台电机 (地址0x01)
 *              - Y轴(深度): 2台电机同步 (地址0x02, 0x03)
 *              - Z轴(高度): 1台电机 (地址0x04)
 *              - 通信: RS485总线, Emm_V5协议, 115200bps
 *              - 编码器: 每台电机自带闭环编码器
 *              - 掉电保持: 电机内部位置保存
 *
 * 配置文件按模块分类：
 * [1] 3D打印机轴配置 - 3轴4电机地址映射
 * [2] 张大头电机参数 - Emm_V5闭环步进电机
 * [3] RS485通信 - 硬件通信接口
 * [4] 调试与诊断 - USMART串口调试工具
 * [5] 系统配置 - 时钟、主循环、看门狗
 *
 ****************************************************************************************************
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ====================================================================================
   [1] 3D打印机轴配置 - 3轴4电机地址映射
   ==================================================================================== */

/* 3D打印机轴定义 */
#define PRINTER_AXIS_COUNT          3           /* 轴数量: X, Y, Z */
#define PRINTER_MOTOR_COUNT         4           /* 电机总数: X(1) + Y(2) + Z(1) */

/* X轴配置（宽度轴，单电机）*/
#define AXIS_X_MOTOR_COUNT          1           /* X轴电机数量 */
#define AXIS_X_ADDR                 0x01        /* X轴电机地址 */
#define AXIS_X_DEFAULT_SPEED        800         /* X轴默认速度(RPM) */
#define AXIS_X_DEFAULT_ACC          20          /* X轴默认加速度 */

/* Y轴配置（深度轴，双电机同步）*/
#define AXIS_Y_MOTOR_COUNT          2           /* Y轴电机数量（双电机同步）*/
#define AXIS_Y_ADDR_LEFT            0x02        /* Y轴左侧电机地址 */
#define AXIS_Y_ADDR_RIGHT           0x03        /* Y轴右侧电机地址 */
#define AXIS_Y_DEFAULT_SPEED        600         /* Y轴默认速度(RPM) */
#define AXIS_Y_DEFAULT_ACC          15          /* Y轴默认加速度 */
#define AXIS_Y_SYNC_ENABLE          1           /* Y轴同步运动使能 */

/* Z轴配置（高度轴，单电机）*/
#define AXIS_Z_MOTOR_COUNT          1           /* Z轴电机数量 */
#define AXIS_Z_ADDR                 0x04        /* Z轴电机地址 */
#define AXIS_Z_DEFAULT_SPEED        200         /* Z轴默认速度(RPM, 较慢保证精度) */
#define AXIS_Z_DEFAULT_ACC          10          /* Z轴默认加速度 */

/* 电机地址列表（用于批量操作）*/
#define MOTOR_ADDR_LIST             {AXIS_X_ADDR, AXIS_Y_ADDR_LEFT, AXIS_Y_ADDR_RIGHT, AXIS_Z_ADDR}

/* 3D打印机机械参数（丝杠传动系统）*/
#define LEAD_SCREW_PITCH_MM         20.0f       /* 丝杠导程: 20mm/圈（电机转1圈，平台移动20mm）*/
#define PULSES_PER_MM               (MOTOR_PULSE_PER_REV / LEAD_SCREW_PITCH_MM)  /* 脉冲/mm: 3200/20 = 160 */
#define MM_PER_PULSE                (LEAD_SCREW_PITCH_MM / MOTOR_PULSE_PER_REV)  /* mm/脉冲: 20/3200 = 0.00625 */
#define THEORETICAL_RESOLUTION_MM   0.00625f    /* 理论分辨率: 0.00625mm/脉冲 (1/160) */
#define MECHANICAL_ACCURACY_MM      0.02f       /* 直线模组装配精度: ±0.02mm（机械累积误差）*/

/* Y_V2协议位置换算系数（X固件使用角度而非脉冲）*/
#define DEGREES_PER_MM              (360.0f / LEAD_SCREW_PITCH_MM)  /* 角度/mm: 360/20 = 18.0° */
#define MM_PER_DEGREE               (LEAD_SCREW_PITCH_MM / 360.0f)  /* mm/角度: 20/360 = 0.0556mm */
#define PULSES_TO_DEGREES(pulses)   ((pulses) * 360.0f / MOTOR_PULSE_PER_REV)  /* 脉冲→角度: pulses/3200*360 */
#define DEGREES_TO_PULSES(degrees)  ((degrees) * MOTOR_PULSE_PER_REV / 360.0f)  /* 角度→脉冲: degrees/360*3200 */

/* 轴行程限制（安全保护，单位mm，相对于原点）*/
#define AXIS_X_MIN_POS_MM           -5600.0f    /* X轴最小位置: -5600mm（负方向极限）*/
#define AXIS_X_MAX_POS_MM           5600.0f     /* X轴最大位置: +5600mm（正方向极限）*/
#define AXIS_Y_MIN_POS_MM           0.0f        /* Y轴最小位置: 0mm（原点位置）*/
#define AXIS_Y_MAX_POS_MM           300.0f      /* Y轴最大位置: 300mm（行程终点）*/
#define AXIS_Z_MIN_POS_MM           0.0f        /* Z轴最小位置: 0mm（原点位置）*/
#define AXIS_Z_MAX_POS_MM           400.0f      /* Z轴最大位置: 400mm（行程终点）*/

/* 轴行程定义（用于兼容旧代码）*/
#define AXIS_X_MAX_TRAVEL_MM        (AXIS_X_MAX_POS_MM - AXIS_X_MIN_POS_MM)  /* X轴总行程 */
#define AXIS_Y_MAX_TRAVEL_MM        (AXIS_Y_MAX_POS_MM - AXIS_Y_MIN_POS_MM)  /* Y轴总行程 */
#define AXIS_Z_MAX_TRAVEL_MM        (AXIS_Z_MAX_POS_MM - AXIS_Z_MIN_POS_MM)  /* Z轴总行程 */

/* ====================================================================================
   [2] 张大头电机(ZDT Motor)配置 - Emm_V5.0闭环步进电机
   ==================================================================================== */

/* 电机基本参数 */
#define MOTOR_DEFAULT_ADDRESS       0x01        /* 默认电机地址（已由轴配置取代）*/
#define MOTOR_DEFAULT_SPEED         300         /* 默认速度(RPM): 0-5000 */
#define MOTOR_DEFAULT_ACC           10          /* 默认加速度: 0-255 */
#define MOTOR_DEFAULT_DIR           0           /* 默认方向: 0=CW顺时针, 1=CCW逆时针 */

/* 电机机械参数（张大头Y系列闭环步进电机）*/
#define MOTOR_SUBDIVISION           16          /* 细分数: 16细分 */
#define MOTOR_PULSE_PER_REV         3200        /* 单圈脉冲数: 16细分时3200脉冲/圈 */
#define MOTOR_HALF_REV_PULSE        1600        /* 半圈脉冲数 */

/* 电机运动模式 */
#define MOTOR_MODE_RELATIVE         false       /* 相对运动模式 */
#define MOTOR_MODE_ABSOLUTE         true        /* 绝对运动模式 */

/* 回零参数 */
#define MOTOR_HOME_MODE             0           /* 回零模式: 0=单圈就近, 1=单圈方向, 2=多圈无限位, 3=多圈限位 */
#define MOTOR_HOME_SPEED            200         /* 回零速度(RPM) */

/* 电机通信协议（Emm_V5.0 - 张大头闭环伺服专用协议） */
/* 协议特点: 地址+功能码+数据+固定校验0x6B */
#define MOTOR_COMM_BAUDRATE         115200      /* 电机通信波特率 */
#define MOTOR_COMM_TIMEOUT_MS       1000        /* 命令响应超时(ms) */
#define MOTOR_RESPONSE_TIMEOUT_MS   500         /* 应用层响应超时(ms) - Phase 2新增 */

/* 电机多机扩展 */
#define MAX_MOTOR_COUNT             4           /* 当前系统电机数量 */
#define MOTOR_FRAME_TIMEOUT_MS      100         /* 帧间隔超时(ms) */

/* 电机功能开关 */
#define MOTOR_ENABLE_CONTROL        1           /* 1=启用电机控制, 0=禁用 */
#define MOTOR_MULTI_AXIS            1           /* 1=启用多轴控制, 0=单轴（3D打印机必须启用）*/
#define MOTOR_SYNC_MOTION           1           /* 1=启用同步运动, 0=独立运动（Y轴双电机需要）*/
#define MOTOR_AUTO_HOME             0           /* 1=启动时自动回零, 0=手动回零 */
#define MOTOR_CLOG_CHECK            1           /* 1=启用堵转检测, 0=禁用 */

/* 3D打印机功能配置 */
#define PRINTER_HOME_ON_STARTUP     0           /* 1=开机自动回零所有轴, 0=手动回零 */
#define PRINTER_ENABLE_ALL_ON_START 1           /* 1=开机自动使能所有电机, 0=手动使能 */
#define PRINTER_SYNC_Y_AXIS         1           /* 1=Y轴双电机强制同步, 0=独立控制 */

/* ====================================================================================
   [3] RS485通信配置 - 硬件物理层
   ==================================================================================== */

/* USART2硬件配置（RS485物理接口） */
#define RS485_USART                 USART2      /* 使用USART2作为RS485接口 */
#define RS485_BAUDRATE              115200      /* RS485波特率 */
#define RS485_TIMEOUT_MS            1000        /* 硬件发送超时(ms) */
#define RS485_RX_BUFFER_SIZE        256         /* 接收FIFO大小 */

/* ====================================================================================
   [4] 调试与诊断配置
   ==================================================================================== */

/* USMART串口调试工具 */
#ifdef NDEBUG
    #define FEATURE_USMART_ENABLE   0           /* Release: 禁用以减少Flash */
#else
    #define FEATURE_USMART_ENABLE   1           /* Debug: 启用 */
#endif



/* ====================================================================================
   [5] 系统配置 - 时钟、循环、看门狗
   ==================================================================================== */

/* 主循环参数 */
#define MAIN_LOOP_DELAY_MS          10          /* 主循环延时(ms) */
#define LED_HEARTBEAT_PERIOD_MS     500         /* LED心跳周期(ms) */

/* 按键配置 */
#define KEY_SCAN_PERIOD_MS          10          /* 按键扫描周期(ms) */
#define KEY_ACTION_DELAY_MS         10          /* 按键动作后延时(ms) */

/* 系统时钟 */
#define SYSTEM_CLOCK_MHZ            72          /* 系统时钟频率(MHz) */
#define HSE_CLOCK_MHZ               8           /* 外部晶振频率(MHz) */
#define PLL_MULTIPLIER              RCC_PLL_MUL9 /* PLL倍频系数: 8MHz * 9 = 72MHz */

/* 调试串口配置（USART1） */
#define DEBUG_UART_BAUDRATE         115200      /* 调试串口波特率 */
#ifdef NDEBUG
    #define DEBUG_UART_ENABLE       0           /* Release: 禁用printf */
#else
    #define DEBUG_UART_ENABLE       1           /* Debug: 启用printf调试 */
#endif

/* 看门狗配置（V3.5 Phase 2启用） */
#define WATCHDOG_ENABLE             0           /* ❌ 暂时禁用（导致系统每2秒复位，待调试主循环性能）*/

/* 电机监控调试开关 */
#define MOTOR_MONITOR_DEBUG         1           /* 0=禁用查询日志, 1=启用（减少串口负载）*/

/* 说明: 条件编译开关汇总
 * - FEATURE_USMART_ENABLE: USMART调试组件 (Release自动禁用)
 * - FEATURE_MODBUS_ENABLE: Modbus RTU协议栈 (手动配置)
 * - FEATURE_MULTI_MOTOR_BATCH_ENABLE: 多电机批处理函数 (手动配置)
 * - WATCHDOG_ENABLE: 看门狗保护 (V3.5 Phase 2启用，当前禁用待调试)
 * - MOTOR_MONITOR_DEBUG: 电机监控查询日志 (调试时启用，生产环境禁用)
 */

#if WATCHDOG_ENABLE
#define IWDG_TIMEOUT_MS             2000        /* 看门狗超时时间(ms): 2s超时保护 */
#define IWDG_FEED_PERIOD_MS         500         /* 喂狗周期(ms): 主循环每次执行喂狗 */
#endif

/* ====================================================================================
   [5] 调试功能配置 - 日志、诊断开关
   ==================================================================================== */

/* 调试总开关 (Release构建自动禁用) */
#ifdef NDEBUG
    #define DEBUG_ENABLE            0           /* Release: 禁用调试功能 */
#else
    #define DEBUG_ENABLE            1           /* Debug: 启用调试功能 */
#endif

/* 模块调试开关 */
#define DEBUG_MOTOR_RESPONSE        0           /* 1=打印电机响应帧, 0=关闭 */
#define DEBUG_KEY_EVENT             0           /* 1=打印按键事件, 0=关闭 */
#define DEBUG_SYSTEM_INFO           0           /* 1=启动时打印系统信息, 0=关闭 */
#define DEBUG_WATCHDOG_FEED         0           /* 1=打印喂狗事件, 0=关闭 */

/* 日志等级 (已废弃，使用logger.h中的统一定义) */
/* #define LOG_LEVEL_ERROR             1 */
/* #define LOG_LEVEL_WARNING           2 */
/* #define LOG_LEVEL_INFO              3 */
/* #define LOG_LEVEL_DEBUG             4 */
/* #define CURRENT_LOG_LEVEL           LOG_LEVEL_INFO */

/* ====================================================================================
   辅助宏定义 - 兼容性定义
   ==================================================================================== */

/* 注意: 日志宏定义已移至 log_system.h，此处不再重复定义 */

/* 电机状态检查宏（V3.5 Phase 4实现） */
/* 注意: 使用前需#include "modbus_gateway.h" */
#define IS_MOTOR_ENABLED(addr)      modbus_gateway_is_motor_enabled(addr)
#define IS_MOTOR_READY(addr)        modbus_gateway_is_motor_ready(addr)

/* 兼容性定义（避免修改主程序代码） */
#define FEATURE_WATCHDOG_ENABLE     WATCHDOG_ENABLE

/* ====================================================================================
   [6] 实时性能优化配置（V3.6新增 - 微秒级响应）
   ==================================================================================== */

/* 实时电机控制模式 */
#define REALTIME_MOTOR_ENABLE       1           /* 1=启用微秒级实时控制, 0=使用标准模式 */

#if REALTIME_MOTOR_ENABLE

/* 实时控制参数（仅在REALTIME_MOTOR_ENABLE=1时生效） */
#define RT_MOTOR_TICK_US            100         /* 定时器周期（微秒）：100μs = 10kHz控制频率 */
#define RT_CMD_QUEUE_SIZE           32          /* 命令队列深度：32个命令 = 512B RAM */
#define RT_USE_DMA                  1           /* 1=DMA非阻塞发送, 0=中断发送 */
#define RT_ENABLE_PROFILING         1           /* 1=启用性能统计, 0=禁用 */
#define RT_DEBUG_PRINTF             0           /* 1=启用printf调试（+200μs延迟）, 0=禁用 */
#define RT_USE_PRECALC_CMDS         0           /* 1=启用预编译命令（需18KB ROM+30KB RAM）, 0=动态构造（+5μs） */

/* 性能目标 */
#define RT_TARGET_LATENCY_US        10          /* 目标延迟：10μs（API调用到DMA启动） */
#define RT_TARGET_THROUGHPUT_CPS    8000        /* 目标吞吐量：8000 cmd/s */

/* 实时模式使用说明 */
/* 
 * 启用实时模式后，需要：
 * 1. CubeMX中启用USART2的DMA（DMA1 Channel 7, Priority=Very High）
 * 2. 配置TIM2定时器为100μs周期中断（可选，用于轨迹插补）
 * 3. 在main.c中调用rt_motor_init()替代原有初始化
 * 4. 使用RT_MOTOR_*宏替代Emm_V5_*函数（详见realtime_motor.h）
 * 
 * 性能提升：
 * - 命令发送延迟：从8.68μs/字节 → <10μs（15x提升）
 * - 主循环响应：从200ms → 100μs（2000x提升）
 * - 命令吞吐量：从500 cmd/s → 8000 cmd/s（16x提升）
 * 
 * 内存开销：
 * - Flash: +18KB（预编译命令缓存）
 * - RAM: +512B（命令队列）
 */

#endif /* REALTIME_MOTOR_ENABLE */

#endif /* __APP_CONFIG_H */
