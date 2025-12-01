/**
 ******************************************************************************
 * @file    modbus_gateway.h
 * @author  STM32_485 Project
 * @version V1.0
 * @date    2025-12-01
 * @brief   Modbus寄存器映射与电机控制网关头文件
 *          将Modbus寄存器映射到ZDT电机控制命令
 ******************************************************************************
 * @attention
 * 本模块实现Modbus寄存器与ZDT电机控制的映射关系，包括：
 * - 寄存器存储（全局控制区、电机控制区、电机状态区）
 * - 寄存器读写API
 * - EXEC_COMMAND命令解析与执行
 * - Modbus → Emm_V5 API映射
 * - 电机状态轮询与更新
 ******************************************************************************
 */

#ifndef __MODBUS_GATEWAY_H
#define __MODBUS_GATEWAY_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================== 配置参数 ======================== */

/* 最大电机数量由 app_config.h 的 MODBUS_MAX_MOTORS 定义 */
#define MODBUS_REGS_PER_MOTOR           64      /* 每个电机寄存器数 */

/* ======================== 寄存器地址定义 ======================== */

/* 全局控制区（0x0000-0x007F, 128寄存器） */
#define MODBUS_REG_GLOBAL_BASE          0x0000

/* 系统配置子区（0x0000-0x000F） */
#define REG_SYS_ENABLE                  0x0000  /* 系统使能（BIT0-15对应16个电机） */
#define REG_SYS_STATUS                  0x0001  /* 系统状态 */
#define REG_SYS_ERROR_CODE              0x0002  /* 系统错误码 */
#define REG_SYS_RESET                   0x0003  /* 系统复位（写0xAA55触发） */
#define REG_SYS_SAVE_PARAMS             0x0004  /* 保存参数到Flash（写0x5A5A） */
#define REG_SYS_LOAD_PARAMS             0x0005  /* 从Flash加载参数 */
#define REG_SYS_FACTORY_RESET           0x0006  /* 恢复出厂设置（写0x1234） */
#define REG_SYS_FIRMWARE_VERSION        0x0007  /* 固件版本（只读） */
#define REG_SYS_HARDWARE_VERSION        0x0008  /* 硬件版本（只读） */
#define REG_SYS_DEVICE_ID               0x0009  /* 设备ID（Modbus地址） */
#define REG_SYS_BAUDRATE                0x000A  /* 波特率配置 */
#define REG_SYS_PROTOCOL                0x000B  /* 协议选择 */
#define REG_HEARTBEAT_TIMEOUT           0x000C  /* 心跳超时时间(ms) */
#define REG_WATCHDOG_TIMEOUT            0x000D  /* 看门狗超时时间(ms) */

/* 电机控制区（0x0100-0x04FF, 16电机×64寄存器） */
#define MODBUS_REG_MOTOR_CTRL_BASE      0x0100

/* 电机控制寄存器偏移（相对电机基地址） */
#define MOTOR_REG_ENABLE                0       /* 使能 */
#define MOTOR_REG_CTRL_MODE             1       /* 控制模式 */
#define MOTOR_REG_DIRECTION             2       /* 方向 */
#define MOTOR_REG_SPEED                 3       /* 速度(0.1RPM) */
#define MOTOR_REG_ACCELERATION          4       /* 加速度 */
#define MOTOR_REG_DECELERATION          5       /* 减速度 */
#define MOTOR_REG_POSITION_H            6       /* 位置高16位 */
#define MOTOR_REG_POSITION_L            7       /* 位置低16位 */
#define MOTOR_REG_EXEC_COMMAND          8       /* 执行命令 */
#define MOTOR_REG_SYNC_FLAG             9       /* 同步标志 */
#define MOTOR_REG_MOTION_TYPE           10      /* 运动类型 */
#define MOTOR_REG_CURRENT_LIMIT         11      /* 电流限制(mA) */
#define MOTOR_REG_HOME_MODE             12      /* 回零模式(0-3) V3.5 Phase 4 */
#define MOTOR_REG_HOME_SPEED            13      /* 回零速度(RPM) V3.5 Phase 4 */
#define MOTOR_REG_HOME_TIMEOUT          14      /* 回零超时(ms) V3.5 Phase 4 */

/* 电机状态区（0x0500-0x08FF, 16电机×64寄存器，只读） */
#define MODBUS_REG_MOTOR_STATUS_BASE    0x0500

/* 电机状态寄存器偏移 */
#define MOTOR_STATUS_REAL_POSITION_H    0       /* 实时位置高16位 */
#define MOTOR_STATUS_REAL_POSITION_L    1       /* 实时位置低16位 */
#define MOTOR_STATUS_REAL_SPEED         2       /* 实时速度 */
#define MOTOR_STATUS_REAL_CURRENT       3       /* 实时电流 */
#define MOTOR_STATUS_REAL_VOLTAGE       4       /* 实时电压 */
#define MOTOR_STATUS_REAL_TEMP          5       /* 实时温度 */
#define MOTOR_STATUS_FLAGS              16      /* 状态标志 */
#define MOTOR_STATUS_ERROR_CODE         22      /* 错误码 */

/* Coils区（0x0C00-0x0C1F, 32个线圈） */
#define MODBUS_COIL_BASE                0x0C00
#define COIL_MOTOR1_ENABLE              0x0C00  /* 电机1使能 */
#define COIL_EMERGENCY_STOP             0x0C10  /* 紧急停止 */
#define COIL_SYNC_MOTION_TRIGGER        0x0C14  /* 同步运动触发 */

/* Discrete Inputs区（0x0C20-0x0C3F, 32个离散输入，只读） */
#define MODBUS_DISCRETE_BASE            0x0C20
#define DISCRETE_MOTOR1_ARRIVED         0x0C20  /* 电机1到位 */
#define DISCRETE_SYSTEM_FAULT           0x0C30  /* 系统故障 */

/* ======================== 命令码定义 ======================== */

/* EXEC_COMMAND命令码 */
typedef enum {
    MOTOR_CMD_ENABLE = 1,           /* 使能电机 */
    MOTOR_CMD_DISABLE = 2,          /* 失能电机 */
    MOTOR_CMD_POS_MOVE = 3,         /* 位置运动 */
    MOTOR_CMD_VEL_MOVE = 4,         /* 速度运动 */
    MOTOR_CMD_STOP = 5,             /* 立即停止 */
    MOTOR_CMD_HOMING = 6,           /* 回零 */
    MOTOR_CMD_CLEAR_POS = 7,        /* 清零位置 */
    MOTOR_CMD_CLEAR_PROTECT = 8,    /* 解除保护 */
} motor_command_t;

/* 控制模式 */
typedef enum {
    CTRL_MODE_TORQUE = 0,           /* 力矩模式（X固件） */
    CTRL_MODE_SPEED = 1,            /* 速度模式 */
    CTRL_MODE_POSITION_DIRECT = 2,  /* 位置直通模式 */
    CTRL_MODE_POSITION_TRAPEZOID = 3, /* 位置梯形曲线模式 */
} motor_ctrl_mode_t;

/* 运动类型 */
typedef enum {
    MOTION_TYPE_RELATIVE_CURRENT = 0,  /* 相对当前位置 */
    MOTION_TYPE_RELATIVE_LAST = 1,     /* 相对上次目标 */
    MOTION_TYPE_ABSOLUTE = 2,          /* 绝对位置 */
} motor_motion_type_t;

/* ======================== 数据结构定义 ======================== */

/**
 * @brief 电机控制寄存器结构体（写入参数）
 */
typedef struct {
    uint16_t enable;                /* 使能标志 */
    uint16_t ctrl_mode;             /* 控制模式 */
    uint16_t direction;             /* 方向 */
    uint16_t speed;                 /* 速度 */
    uint16_t acceleration;          /* 加速度 */
    uint16_t deceleration;          /* 减速度 */
    uint16_t position_h;            /* 位置高16位 */
    uint16_t position_l;            /* 位置低16位 */
    uint16_t exec_command;          /* 执行命令 */
    uint16_t sync_flag;             /* 同步标志 */
    uint16_t motion_type;           /* 运动类型 */
    uint16_t current_limit;         /* 电流限制 */
    uint16_t home_mode;             /* 回零模式(0-3) V3.5 Phase 4 */
    uint16_t home_speed;            /* 回零速度(RPM) V3.5 Phase 4 */
    uint16_t home_timeout;          /* 回零超时(ms) V3.5 Phase 4 */
    /* 更多参数... */
    uint16_t reserved[49];          /* 预留空间（共64寄存器） */
} motor_control_regs_t;

/**
 * @brief 电机状态寄存器结构体（读取反馈）
 */
typedef struct {
    uint16_t real_position_h;       /* 实时位置高16位 */
    uint16_t real_position_l;       /* 实时位置低16位 */
    uint16_t real_speed;            /* 实时速度 */
    uint16_t real_current;          /* 实时电流 */
    uint16_t real_voltage;          /* 实时电压 */
    uint16_t real_temp;             /* 实时温度 */
    uint16_t reserved1[10];
    uint16_t status_flags;          /* 状态标志 */
    uint16_t reserved2[5];
    uint16_t error_code;            /* 错误码 */
    /* 更多状态... */
    uint16_t reserved3[41];         /* 预留空间（共64寄存器） */
} motor_status_regs_t;

/**
 * @brief 全局控制寄存器结构体
 */
typedef struct {
    uint16_t sys_enable;            /* 系统使能 */
    uint16_t sys_status;            /* 系统状态 */
    uint16_t sys_error_code;        /* 系统错误码 */
    uint16_t sys_reset;             /* 系统复位 */
    uint16_t sys_save_params;       /* 保存参数 */
    uint16_t sys_load_params;       /* 加载参数 */
    uint16_t sys_factory_reset;     /* 恢复出厂 */
    uint16_t firmware_version;      /* 固件版本 */
    uint16_t hardware_version;      /* 硬件版本 */
    uint16_t device_id;             /* 设备ID */
    uint16_t baudrate_index;        /* 波特率索引（0-8对应9600-921600） */
    uint16_t protocol;              /* 协议选择 */
    uint16_t heartbeat_timeout;     /* 心跳超时 */
    uint16_t watchdog_timeout;      /* 看门狗超时 */
    uint16_t reserved[114];         /* 预留空间（共128寄存器） */
} global_control_regs_t;

/**
 * @brief 线圈状态结构体（位操作）
 */
typedef struct {
    uint8_t coils[4];               /* 32个线圈（32位） */
} coils_t;

/**
 * @brief 离散输入状态结构体（位操作，只读）
 */
typedef struct {
    uint8_t inputs[4];              /* 32个离散输入（32位） */
} discrete_inputs_t;

/* ======================== 函数声明 ======================== */

/**
 * @brief  Modbus网关初始化
 * @param  None
 * @retval 0=成功, -1=失败
 */
int modbus_gateway_init(void);

/**
 * @brief  读保持寄存器（功能码0x03）
 * @param  start_addr: 起始地址
 * @param  num_regs: 寄存器数量
 * @param  data: 输出数据缓冲区（大端序，每个寄存器2字节）
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_read_holding_registers(uint16_t start_addr, uint16_t num_regs, uint8_t *data);

/**
 * @brief  写单个寄存器（功能码0x06）
 * @param  reg_addr: 寄存器地址
 * @param  reg_value: 寄存器值
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_write_single_register(uint16_t reg_addr, uint16_t reg_value);

/**
 * @brief  写多个寄存器（功能码0x10）
 * @param  start_addr: 起始地址
 * @param  num_regs: 寄存器数量
 * @param  data: 输入数据缓冲区（大端序）
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_write_multiple_registers(uint16_t start_addr, uint16_t num_regs, const uint8_t *data);

/**
 * @brief  读输入寄存器（功能码0x04）
 * @param  start_addr: 起始地址
 * @param  num_regs: 寄存器数量
 * @param  data: 输出数据缓冲区
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_read_input_registers(uint16_t start_addr, uint16_t num_regs, uint8_t *data);

/**
 * @brief  读线圈（功能码0x01）
 * @param  start_addr: 起始地址
 * @param  num_coils: 线圈数量
 * @param  data: 输出数据缓冲区
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_read_coils(uint16_t start_addr, uint16_t num_coils, uint8_t *data);

/**
 * @brief  读离散输入（功能码0x02）
 * @param  start_addr: 起始地址
 * @param  num_inputs: 输入数量
 * @param  data: 输出数据缓冲区
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_read_discrete_inputs(uint16_t start_addr, uint16_t num_inputs, uint8_t *data);

/**
 * @brief  写单个线圈（功能码0x05）
 * @param  coil_addr: 线圈地址
 * @param  coil_value: 线圈值（0x0000或0xFF00）
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_write_single_coil(uint16_t coil_addr, uint16_t coil_value);

/**
 * @brief  写多个线圈（功能码0x0F）
 * @param  start_addr: 起始地址
 * @param  num_coils: 线圈数量
 * @param  data: 输入数据缓冲区
 * @retval 0=成功, 其他=异常码
 */
int modbus_gateway_write_multiple_coils(uint16_t start_addr, uint16_t num_coils, const uint8_t *data);

/**
 * @brief  周期性更新电机状态（主循环调用）
 * @param  None
 * @retval None
 * @note   轮询更新电机实时位置、速度、电流等状态寄存器
 */
void modbus_gateway_update_motor_status(void);

/**
 * @brief  执行电机控制命令（写入EXEC_COMMAND触发）
 * @param  motor_id: 电机ID（1-16）
 * @param  command: 命令码
 * @retval 0=成功, 其他=错误码
 */
int modbus_gateway_execute_motor_command(uint8_t motor_id, motor_command_t command);

/**
 * @brief  获取电机控制寄存器指针
 * @param  motor_id: 电机ID（1-16）
 * @retval 寄存器指针，失败返回NULL
 */
motor_control_regs_t* modbus_gateway_get_motor_control_regs(uint8_t motor_id);

/**
 * @brief  获取电机状态寄存器指针
 * @param  motor_id: 电机ID（1-16）
 * @retval 寄存器指针，失败返回NULL
 */
motor_status_regs_t* modbus_gateway_get_motor_status_regs(uint8_t motor_id);

/**
 * @brief  获取全局控制寄存器指针
 * @param  None
 * @retval 寄存器指针
 */
global_control_regs_t* modbus_gateway_get_global_regs(void);

/**
 * @brief  检查电机是否使能（V3.5 Phase 4新增）
 * @param  motor_id: 电机ID（1-16）
 * @retval 1=使能, 0=失能或参数错误
 */
uint8_t modbus_gateway_is_motor_enabled(uint8_t motor_id);

/**
 * @brief  检查电机是否就绪（V3.5 Phase 4新增）
 * @param  motor_id: 电机ID（1-16）
 * @retval 1=就绪, 0=未就绪或参数错误
 */
uint8_t modbus_gateway_is_motor_ready(uint8_t motor_id);

/**
 * @brief  处理电机查询响应（V3.5 Phase 5新增）
 * @param  motor_addr: 电机地址（1-8）
 * @param  data: 响应帧数据
 * @param  len: 响应帧长度
 * @retval None
 * @note   在USART2 IDLE中断中调用，更新电机状态缓存
 */
void modbus_gateway_handle_motor_response(uint8_t motor_addr, const uint8_t *data, uint16_t len);

/* ======================== 调试宏定义 ======================== */

#if defined(DEBUG) && (DEBUG == 1)
    #define GATEWAY_DEBUG_PRINTF(fmt, ...)   printf("[GATEWAY] " fmt, ##__VA_ARGS__)
#else
    #define GATEWAY_DEBUG_PRINTF(fmt, ...)   ((void)0)
#endif

#endif /* __MODBUS_GATEWAY_H */
