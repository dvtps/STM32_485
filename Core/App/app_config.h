/**
 ****************************************************************************************************
 * @file        app_config.h
 * @author      STM32_485项目
 * @version     V2.0
 * @date        2025-12-01
 * @brief       应用层统一配置文件
 ****************************************************************************************************
 * @attention
 *
 * 配置文件按模块分类：
 * [1] 张大头电机(ZDT Motor) - 闭环步进电机控制
 * [2] RS485通信 - 硬件通信接口
 * [3] Modbus协议 - 应用层通信协议（预留）
 * [4] 系统配置 - 时钟、主循环、看门狗等
 * [5] 调试功能 - 日志、诊断开关
 *
 ****************************************************************************************************
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ====================================================================================
   [1] 张大头电机(ZDT Motor)配置 - Emm_V5.0闭环步进电机
   ==================================================================================== */

/* 电机基本参数 */
#define MOTOR_DEFAULT_ADDRESS       0x01        /* 默认电机地址 */
#define MOTOR_DEFAULT_SPEED         300         /* 默认速度(RPM): 0-5000 */
#define MOTOR_DEFAULT_ACC           10          /* 默认加速度: 0-255 */
#define MOTOR_DEFAULT_DIR           0           /* 默认方向: 0=CW顺时针, 1=CCW逆时针 */

/* 电机机械参数 */
#define MOTOR_SUBDIVISION           16          /* 细分数: 16细分 */
#define MOTOR_PULSE_PER_REV         3200        /* 单圈脉冲数: 16细分时3200脉冲/圈 */
#define MOTOR_HALF_REV_PULSE        1600        /* 半圈脉冲数 */

/* 电机运动模式 */
#define MOTOR_MODE_RELATIVE         false       /* 相对运动模式 */
#define MOTOR_MODE_ABSOLUTE         true        /* 绝对运动模式 */

/* 回零参数 */
#define MOTOR_HOME_MODE             0           /* 回零模式: 0=单圈就近, 1=单圈方向, 2=多圈无限位, 3=多圈限位 */
#define MOTOR_HOME_SPEED            200         /* 回零速度(RPM) */

/* 电机通信协议（Emm_V5.0 - 基于Modbus-Like协议，厂商定制） */
/* 协议特点: Modbus帧格式 + 固定校验0x6B + 厂商专用功能码 */
#define MOTOR_COMM_BAUDRATE         115200      /* 电机通信波特率（固定115200） */
#define MOTOR_COMM_TIMEOUT_MS       1000        /* 命令响应超时(ms) */
#define MOTOR_RESPONSE_TIMEOUT_MS   500         /* 应用层响应超时(ms) - Phase 2新增 */

/* 电机多机扩展 */
#define MAX_MOTOR_COUNT             4           /* 支持的最大电机数量（1-255地址可用） */
#define MOTOR_FRAME_TIMEOUT_MS      100         /* 帧间隔超时(ms) */

/* 电机功能开关 */
#define MOTOR_ENABLE_CONTROL        1           /* 1=启用电机控制, 0=禁用 */
#define MOTOR_MULTI_AXIS            0           /* 1=启用多轴控制, 0=单轴 */
#define MOTOR_SYNC_MOTION           0           /* 1=启用同步运动, 0=独立运动 */
#define MOTOR_AUTO_HOME             0           /* 1=启动时自动回零, 0=手动回零 */
#define MOTOR_CLOG_CHECK            1           /* 1=启用堵转检测, 0=禁用 */

/* 多轴配置（需MOTOR_MULTI_AXIS=1） */
#if MOTOR_MULTI_AXIS
#define MOTOR_AXIS_COUNT            2           /* 电机轴数: 1-255 */
#define MOTOR_ADDR_LIST             {0x01, 0x02} /* 电机地址列表 */
#endif

/* ====================================================================================
   [2] RS485通信配置 - 硬件物理层
   ==================================================================================== */

/* USART2硬件配置（RS485物理接口） */
#define RS485_USART                 USART2      /* 使用USART2作为RS485接口 */
#define RS485_BAUDRATE              115200      /* RS485波特率 */
#define RS485_TIMEOUT_MS            1000        /* 硬件发送超时(ms) */
#define RS485_RX_BUFFER_SIZE        256         /* 接收FIFO大小 */

/* ====================================================================================
   [3] 标准Modbus协议配置 - 用于上位机/PLC通信
   ==================================================================================== */
/* 
 * 说明: 
 * - 张大头电机使用 Emm_V5.0 协议（类Modbus帧格式 + 厂商功能码 + 固定校验0x6B）
 * - 此处配置的是"标准Modbus RTU"协议，通过寄存器映射控制电机
 * - 架构设计：
 *   → USART2: 电机直接通信 (Emm_V5协议) - BSP层
 *   → Modbus RTU: 上位机通信 (标准协议) - Middlewares层
 *   → Gateway: Modbus寄存器 → 电机命令映射 - App层
 */

#define FEATURE_MODBUS_ENABLE       1           /* 1=启用Modbus RTU功能, 0=禁用 */

/* USMART串口调试工具 (Release构建可禁用以减少Flash占用) */
#ifdef NDEBUG
    #define FEATURE_USMART_ENABLE   0           /* Release: 禁用USMART (~2KB Flash优化) */
#else
    #define FEATURE_USMART_ENABLE   1           /* Debug: 启用USMART调试工具 */
#endif

/* 多电机批处理函数 (按需启用) */
#define FEATURE_MULTI_MOTOR_BATCH_ENABLE  1     /* 1=启用批处理API (2.6KB), 0=禁用 */

#if FEATURE_MODBUS_ENABLE
#define MODBUS_SLAVE_ADDRESS        1           /* Modbus从机地址: 1-247 */
#define MODBUS_BAUDRATE             115200      /* 波特率: 9600/19200/115200 */
#define MODBUS_RESPONSE_TIMEOUT_MS  200         /* 响应超时(ms) */
#define MODBUS_MAX_MOTORS           MAX_MOTOR_COUNT /* 最大支持电机数量(使用统一配置) */
#define MODBUS_FRAME_TIMEOUT_MS     50          /* 帧间隔超时(ms) */
#endif

/* ====================================================================================
   [4] 系统配置 - 时钟、循环、看门狗
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

/* 看门狗配置 */
#define WATCHDOG_ENABLE             0           /* 1=启用看门狗, 0=禁用 (Modbus测试时临时禁用) */

/* 说明: 条件编译开关汇总
 * - FEATURE_USMART_ENABLE: USMART调试组件 (Release自动禁用)
 * - FEATURE_MODBUS_ENABLE: Modbus RTU协议栈 (手动配置)
 * - FEATURE_MULTI_MOTOR_BATCH_ENABLE: 多电机批处理函数 (手动配置)
 */

#if WATCHDOG_ENABLE
#define IWDG_TIMEOUT_MS             2000        /* 看门狗超时时间(ms): 500/1000/2000/5000 */
#define IWDG_FEED_PERIOD_MS         500         /* 喂狗周期(ms): 必须小于超时时间的一半 */
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

/* 日志等级 */
#define LOG_LEVEL_ERROR             1           /* 错误日志 */
#define LOG_LEVEL_WARNING           2           /* 警告日志 */
#define LOG_LEVEL_INFO              3           /* 信息日志 */
#define LOG_LEVEL_DEBUG             4           /* 调试日志 */

#define CURRENT_LOG_LEVEL           LOG_LEVEL_INFO /* 当前日志等级 */

/* ====================================================================================
   辅助宏定义 - 兼容性定义
   ==================================================================================== */

/* 注意: 日志宏定义已移至 log_system.h，此处不再重复定义 */

/* 电机状态检查宏（待实现） */
#define IS_MOTOR_ENABLED(addr)      /* TODO: 检查电机是否使能 */
#define IS_MOTOR_READY(addr)        /* TODO: 检查电机是否就绪 */

/* 兼容性定义（避免修改主程序代码） */
#define FEATURE_WATCHDOG_ENABLE     WATCHDOG_ENABLE

#endif /* __APP_CONFIG_H */
