/**
 ******************************************************************************
 * @file    multi_motor_manager.h
 * @author  STM32_485 Project (Middlewares Layer)
 * @version V3.1
 * @date    2025-12-02
 * @brief   多电机管理中间件 - 支持通过Modbus站号管理1-16台电机
 ******************************************************************************
 * @attention
 * 
 * 架构定位：Middlewares层（可重用中间件）
 * - 不依赖App层业务逻辑
 * - 通过Emm_V5 API调用BSP层
 * - 提供标准化的多电机管理接口
 * - 可被多个App层模块复用
 * 
 * 功能设计：
 * 1. 电机发现：自动扫描RS485总线，识别在线电机
 * 2. 地址映射：Modbus站号 → 电机物理地址（可配置）
 * 3. 批量控制：同时控制多台电机的位置/速度（同步运动）
 * 4. 状态监控：轮询所有在线电机的实时状态
 * 5. 通信统计：每电机独立的tx/rx/timeout/error计数
 * 
 * Modbus寄存器地址映射规则：
 * - 全局区：0x0000-0x007F（128个寄存器）
 * - 电机N控制区：0x0100 + (N-1)*0x40（每电机64寄存器）
 * - 电机N状态区：0x0500 + (N-1)*0x20（每电机32寄存器）
 * 
 ******************************************************************************
 */

#ifndef __MULTI_MOTOR_MANAGER_H
#define __MULTI_MOTOR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/* ======================== 配置参数 ======================== */

#define MULTI_MOTOR_MAX_COUNT       16      /* 最大电机数量（1-16） */
#define MULTI_MOTOR_SCAN_TIMEOUT_MS 100     /* 扫描超时（ms） */
#define MULTI_MOTOR_STATUS_POLL_MS  1000    /* 状态轮询周期（ms） */

/* ======================== 电机状态定义 ======================== */

typedef enum {
    MOTOR_STATE_OFFLINE = 0,    /* 离线 */
    MOTOR_STATE_ONLINE,         /* 在线空闲 */
    MOTOR_STATE_RUNNING,        /* 运行中 */
    MOTOR_STATE_ERROR,          /* 故障 */
} motor_state_t;

typedef struct {
    uint8_t  physical_addr;     /* 物理地址（Emm_V5地址，1-255） */
    uint8_t  modbus_addr;       /* Modbus站号（1-16） */
    bool     is_online;         /* 是否在线 */
    motor_state_t state;        /* 当前状态 */
    
    /* 实时状态（从电机读取） */
    int32_t  current_position;  /* 当前位置（脉冲数） */
    uint16_t current_speed;     /* 当前速度（0.1RPM） */
    uint16_t current_current;   /* 当前电流（mA） */
    uint16_t current_voltage;   /* 当前电压（mV） */
    int16_t  current_temp;      /* 当前温度（℃） */
    
    /* 通信统计 */
    uint32_t tx_count;          /* 发送命令计数 */
    uint32_t rx_count;          /* 接收响应计数 */
    uint32_t timeout_count;     /* 超时计数 */
    uint32_t error_count;       /* 错误计数 */
    
    /* 配置参数 */
    uint16_t max_speed;         /* 最大速度限制（RPM） */
    uint8_t  acceleration;      /* 加速度（1-255） */
    bool     enabled;           /* 使能状态 */
    
} motor_info_t;

/* ======================== 全局API ======================== */

/**
 * @brief       多电机管理器初始化
 * @param       无
 * @retval      0: 成功, -1: 失败
 */
int multi_motor_init(void);

/**
 * @brief       扫描RS485总线，发现在线电机
 * @param       start_addr: 起始地址（1-255）
 * @param       end_addr: 结束地址（1-255）
 * @retval      发现的电机数量
 */
int multi_motor_scan(uint8_t start_addr, uint8_t end_addr);

/**
 * @brief       设置电机地址映射
 * @param       modbus_addr: Modbus站号（1-16）
 * @param       physical_addr: 电机物理地址（1-255）
 * @retval      0: 成功, -1: 失败
 */
int multi_motor_map_address(uint8_t modbus_addr, uint8_t physical_addr);

/**
 * @brief       获取电机信息
 * @param       modbus_addr: Modbus站号（1-16）
 * @retval      电机信息结构体指针，NULL表示无效地址
 */
motor_info_t* multi_motor_get_info(uint8_t modbus_addr);

/**
 * @brief       获取在线电机数量
 * @param       无
 * @retval      在线电机数量
 */
uint8_t multi_motor_get_online_count(void);

/* ======================== 批量控制API ======================== */

/**
 * @brief       批量使能电机
 * @param       motor_mask: 电机掩码（BIT0-15对应1-16号电机）
 * @param       enable: true=使能, false=失能
 * @retval      0: 成功, -1: 失败
 */
int multi_motor_enable_batch(uint16_t motor_mask, bool enable);

/**
 * @brief       批量位置控制（同步运动）
 * @param       motor_mask: 电机掩码
 * @param       dir: 方向（0=CW, 1=CCW）
 * @param       speed: 速度（RPM）
 * @param       acc: 加速度（1-255）
 * @param       pulses: 脉冲数
 * @retval      0: 成功, -1: 失败
 */
int multi_motor_pos_control_batch(uint16_t motor_mask, uint8_t dir, 
                                   uint16_t speed, uint8_t acc, uint32_t pulses);

/**
 * @brief       批量速度控制
 * @param       motor_mask: 电机掩码
 * @param       dir: 方向（0=CW, 1=CCW）
 * @param       speed: 速度（RPM）
 * @param       acc: 加速度（1-255）
 * @retval      0: 成功, -1: 失败
 */
int multi_motor_vel_control_batch(uint16_t motor_mask, uint8_t dir, 
                                   uint16_t speed, uint8_t acc);

/**
 * @brief       批量急停
 * @param       motor_mask: 电机掩码
 * @retval      0: 成功, -1: 失败
 */
int multi_motor_stop_batch(uint16_t motor_mask);

/**
 * @brief       批量回零
 * @param       motor_mask: 电机掩码
 * @param       mode: 回零模式（0-3）
 * @retval      0: 成功, -1: 失败
 */
int multi_motor_home_batch(uint16_t motor_mask, uint8_t mode);

/* ======================== 状态轮询API ======================== */

/**
 * @brief       更新所有在线电机状态（在主循环中调用）
 * @param       无
 * @retval      成功更新的电机数量
 */
int multi_motor_update_status(void);

/**
 * @brief       打印电机列表（调试用）
 * @param       无
 * @retval      无
 */
void multi_motor_print_list(void);

#endif /* __MULTI_MOTOR_MANAGER_H */
