/**
 ******************************************************************************
 * @file    modbus_gateway.c
 * @author  STM32_485 Project
 * @version V1.0
 * @date    2025-12-01
 * @brief   Modbus寄存器映射与电机控制网关实现
 *          实现寄存器存储、读写API、命令解析和电机控制映射
 ******************************************************************************
 */

#include "app_config.h"      /* 必须先包含以获取MODBUS_MAX_MOTORS */
#include "modbus_gateway.h"
#include "modbus_rtu.h"
#include "modbus_hal.h"
#include "error_handler.h"  /* V3.5 Phase 3: 参数验证宏 */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

/* ======================== 私有变量 ======================== */

/* 全局控制寄存器 */
static global_control_regs_t g_global_regs = {
    .firmware_version = 0x0100,     /* V1.0 */
    .hardware_version = 0xF103,     /* STM32F103 */
    .device_id = 1,                 /* 默认地址1 */
    .baudrate_index = 7,            /* 默认115200(索引7) */
    .heartbeat_timeout = 1000,      /* 1s */
    .watchdog_timeout = 2000,       /* 2s */
};

/* 电机控制寄存器数组（16个电机） */
static motor_control_regs_t g_motor_control_regs[MODBUS_MAX_MOTORS] = {0};

/* 电机状态寄存器数组（16个电机） */
static motor_status_regs_t g_motor_status_regs[MODBUS_MAX_MOTORS] = {0};

/* 线圈状态 */
static coils_t g_coils = {0};

/* 离散输入状态 */
static discrete_inputs_t g_discrete_inputs = {0};

/* ======================== 私有函数声明 ======================== */

static uint16_t read_register_by_address(uint16_t addr);
static int write_register_by_address(uint16_t addr, uint16_t value);
static bool is_address_writable(uint16_t addr);

/* ======================== 公共函数实现 ======================== */

/**
 * @brief  Modbus网关初始化
 */
int modbus_gateway_init(void)
{
    /* 清空所有寄存器 */
    memset(g_motor_control_regs, 0, sizeof(g_motor_control_regs));
    memset(g_motor_status_regs, 0, sizeof(g_motor_status_regs));
    memset(&g_coils, 0, sizeof(g_coils));
    memset(&g_discrete_inputs, 0, sizeof(g_discrete_inputs));
    
    /* 初始化全局控制寄存器默认值 */
    g_global_regs.firmware_version = 0x0100;
    g_global_regs.hardware_version = 0xF103;
    g_global_regs.device_id = 1;
    g_global_regs.baudrate_index = 7;  /* 115200 */
    g_global_regs.heartbeat_timeout = 1000;
    g_global_regs.watchdog_timeout = 2000;
    
    /* 初始化电机控制寄存器默认值 */
    for (int i = 0; i < MODBUS_MAX_MOTORS; i++) {
        g_motor_control_regs[i].speed = 1000;           /* 默认100.0 RPM */
        g_motor_control_regs[i].acceleration = 100;     /* 默认加速度 */
        g_motor_control_regs[i].deceleration = 100;     /* 默认减速度 */
        g_motor_control_regs[i].current_limit = 2000;   /* 默认2000mA */
        g_motor_control_regs[i].ctrl_mode = CTRL_MODE_POSITION_TRAPEZOID;
    }
    
    GATEWAY_DEBUG_PRINTF("Modbus网关初始化完成\r\n");
    
    return 0;
}

/**
 * @brief  读保持寄存器（功能码0x03）
 */
int modbus_gateway_read_holding_registers(uint16_t start_addr, uint16_t num_regs, uint8_t *data)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_NULL_PTR(data);
    CHECK_PARAM(num_regs > 0 && num_regs <= 125);  /* Modbus协议限制最大125个寄存器 */
    
    uint16_t i;
    uint16_t reg_value;
    uint16_t end_addr = start_addr + num_regs - 1;
    
    /* 检查地址范围（0x0000-0x08FF有效） */
    if (start_addr > 0x08FF || end_addr > 0x08FF) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 逐个读取寄存器 */
    for (i = 0; i < num_regs; i++) {
        reg_value = read_register_by_address(start_addr + i);
        
        /* 大端序存储 */
        data[i * 2] = (reg_value >> 8) & 0xFF;
        data[i * 2 + 1] = reg_value & 0xFF;
    }
    
    return 0;
}

/**
 * @brief  写单个寄存器（功能码0x06）
 */
int modbus_gateway_write_single_register(uint16_t reg_addr, uint16_t reg_value)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM(reg_addr <= 0x08FF);  /* 地址范围验证 */
    
    /* 检查地址范围 */
    if (reg_addr > 0x08FF) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 检查是否可写 */
    if (!is_address_writable(reg_addr)) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 写入寄存器 */
    return write_register_by_address(reg_addr, reg_value);
}

/**
 * @brief  写多个寄存器（功能码0x10）
 */
int modbus_gateway_write_multiple_registers(uint16_t start_addr, uint16_t num_regs, const uint8_t *data)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_NULL_PTR(data);
    CHECK_PARAM(num_regs > 0 && num_regs <= 123);  /* Modbus协议限制最大123个寄存器 */
    
    uint16_t i;
    uint16_t reg_value;
    int ret;
    
    /* 检查地址范围 */
    if (start_addr + num_regs - 1 > 0x08FF) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 逐个写入寄存器 */
    for (i = 0; i < num_regs; i++) {
        uint16_t addr = start_addr + i;
        
        /* 检查是否可写 */
        if (!is_address_writable(addr)) {
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
        
        /* 大端序解析 */
        reg_value = (data[i * 2] << 8) | data[i * 2 + 1];
        
        /* 写入 */
        ret = write_register_by_address(addr, reg_value);
        if (ret != 0) {
            return ret;
        }
    }
    
    return 0;
}

/**
 * @brief  读输入寄存器（功能码0x04）
 * @note   映射到电机状态区（只读）
 */
int modbus_gateway_read_input_registers(uint16_t start_addr, uint16_t num_regs, uint8_t *data)
{
    uint16_t i;
    uint16_t reg_value;
    
    /* 检查地址范围（0x0500-0x08FF状态区） */
    if (start_addr < 0x0500 || start_addr > 0x08FF || 
        start_addr + num_regs - 1 > 0x08FF) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 逐个读取 */
    for (i = 0; i < num_regs; i++) {
        reg_value = read_register_by_address(start_addr + i);
        data[i * 2] = (reg_value >> 8) & 0xFF;
        data[i * 2 + 1] = reg_value & 0xFF;
    }
    
    return 0;
}

/**
 * @brief  读线圈（功能码0x01）
 */
int modbus_gateway_read_coils(uint16_t start_addr, uint16_t num_coils, uint8_t *data)
{
    uint16_t i, byte_idx, bit_idx;
    uint8_t byte_count = (num_coils + 7) / 8;
    
    /* 检查地址范围（0x0C00-0x0C1F） */
    if (start_addr < 0x0C00 || start_addr > 0x0C1F || 
        start_addr + num_coils - 1 > 0x0C1F) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 清空输出缓冲区 */
    memset(data, 0, byte_count);
    
    /* 逐位读取线圈 */
    for (i = 0; i < num_coils; i++) {
        uint16_t coil_addr = start_addr + i - 0x0C00;
        byte_idx = coil_addr / 8;
        bit_idx = coil_addr % 8;
        
        if (byte_idx < 4 && (g_coils.coils[byte_idx] & (1 << bit_idx))) {
            data[i / 8] |= (1 << (i % 8));
        }
    }
    
    return 0;
}

/**
 * @brief  读离散输入（功能码0x02）
 */
int modbus_gateway_read_discrete_inputs(uint16_t start_addr, uint16_t num_inputs, uint8_t *data)
{
    uint16_t i, byte_idx, bit_idx;
    uint8_t byte_count = (num_inputs + 7) / 8;
    
    /* 检查地址范围（0x0C20-0x0C3F） */
    if (start_addr < 0x0C20 || start_addr > 0x0C3F || 
        start_addr + num_inputs - 1 > 0x0C3F) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 清空输出缓冲区 */
    memset(data, 0, byte_count);
    
    /* 逐位读取 */
    for (i = 0; i < num_inputs; i++) {
        uint16_t input_addr = start_addr + i - 0x0C20;
        byte_idx = input_addr / 8;
        bit_idx = input_addr % 8;
        
        if (byte_idx < 4 && (g_discrete_inputs.inputs[byte_idx] & (1 << bit_idx))) {
            data[i / 8] |= (1 << (i % 8));
        }
    }
    
    return 0;
}

/**
 * @brief  写单个线圈（功能码0x05）
 */
int modbus_gateway_write_single_coil(uint16_t coil_addr, uint16_t coil_value)
{
    uint8_t motor_id;
    uint16_t coil_offset;
    uint8_t byte_idx, bit_idx;
    
    /* 检查地址范围 */
    if (coil_addr < 0x0C00 || coil_addr > 0x0C1F) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    coil_offset = coil_addr - 0x0C00;
    byte_idx = coil_offset / 8;
    bit_idx = coil_offset % 8;
    
    /* 写入线圈 */
    if (coil_value == 0xFF00) {
        g_coils.coils[byte_idx] |= (1 << bit_idx);
    } else {
        g_coils.coils[byte_idx] &= ~(1 << bit_idx);
    }
    
    /* 处理特殊线圈 */
    if (coil_addr >= COIL_MOTOR1_ENABLE && coil_addr < COIL_MOTOR1_ENABLE + MODBUS_MAX_MOTORS) {
        /* 电机使能线圈（0x0C00-0x0C0F） */
        motor_id = coil_addr - COIL_MOTOR1_ENABLE + 1;
        if (motor_id >= 1 && motor_id <= MODBUS_MAX_MOTORS) {
            if (coil_value == 0xFF00) {
                modbus_gateway_execute_motor_command(motor_id, MOTOR_CMD_ENABLE);
            } else {
                modbus_gateway_execute_motor_command(motor_id, MOTOR_CMD_DISABLE);
            }
        }
    } else if (coil_addr == COIL_EMERGENCY_STOP) {
        /* 紧急停止（0x0C10） */
        if (coil_value == 0xFF00) {
            for (motor_id = 1; motor_id <= MODBUS_MAX_MOTORS; motor_id++) {
                modbus_gateway_execute_motor_command(motor_id, MOTOR_CMD_STOP);
            }
        }
    } else if (coil_addr == COIL_SYNC_MOTION_TRIGGER) {
        /* 同步运动触发（0x0C14） */
        if (coil_value == 0xFF00) {
            const modbus_motor_callbacks_t *cb = modbus_get_motor_callbacks();
            if (cb && cb->motor_sync_motion) {
                cb->motor_sync_motion(0);  /* 广播同步命令 */
            }
        }
    }
    
    return 0;
}

/**
 * @brief  写多个线圈（功能码0x0F）
 */
int modbus_gateway_write_multiple_coils(uint16_t start_addr, uint16_t num_coils, const uint8_t *data)
{
    uint16_t i;
    uint16_t coil_value;
    int ret;
    
    /* 检查地址范围 */
    if (start_addr < 0x0C00 || start_addr + num_coils - 1 > 0x0C1F) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    /* 逐个写入线圈 */
    for (i = 0; i < num_coils; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx = i % 8;
        
        coil_value = (data[byte_idx] & (1 << bit_idx)) ? 0xFF00 : 0x0000;
        
        ret = modbus_gateway_write_single_coil(start_addr + i, coil_value);
        if (ret != 0) {
            return ret;
        }
    }
    
    return 0;
}

/**
 * @brief  周期性更新电机状态（主循环调用，暂未实现查询）
 */
void modbus_gateway_update_motor_status(void)
{
    /* TODO: 实现电机状态轮询
     * 调用Emm_V5_Read_Sys_Params()等查询函数
     * 更新g_motor_status_regs数组
     * 更新g_discrete_inputs（到位标志等）
     */
}

/**
 * @brief  执行电机控制命令
 */
int modbus_gateway_execute_motor_command(uint8_t motor_id, motor_command_t command)
{
    motor_control_regs_t *ctrl_regs;
    int32_t position;
    bool sync_flag;
    
    /* 检查电机ID */
    if (motor_id < 1 || motor_id > MODBUS_MAX_MOTORS) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    ctrl_regs = &g_motor_control_regs[motor_id - 1];
    sync_flag = (ctrl_regs->sync_flag == 1);
    
    /* 获取回调函数 */
    const modbus_motor_callbacks_t *cb = modbus_get_motor_callbacks();
    if (cb == NULL) {
        return MODBUS_EX_SLAVE_DEVICE_FAILURE;  /* 未注册回调 */
    }
    
    /* 根据命令码执行 */
    switch (command) {
        case MOTOR_CMD_ENABLE:
            if (cb->motor_enable) {
                cb->motor_enable(motor_id, true, sync_flag);
            }
            GATEWAY_DEBUG_PRINTF("电机%d使能\r\n", motor_id);
            break;
            
        case MOTOR_CMD_DISABLE:
            if (cb->motor_enable) {
                cb->motor_enable(motor_id, false, sync_flag);
            }
            GATEWAY_DEBUG_PRINTF("电机%d失能\r\n", motor_id);
            break;
            
        case MOTOR_CMD_POS_MOVE:
            /* 合并32位位置 */
            position = ((int32_t)ctrl_regs->position_h << 16) | ctrl_regs->position_l;
            
            /* 根据控制模式选择命令 */
            if (ctrl_regs->ctrl_mode == CTRL_MODE_POSITION_TRAPEZOID) {
                if (cb->motor_pos_control) {
                    cb->motor_pos_control(
                        motor_id,
                        ctrl_regs->direction,
                        ctrl_regs->speed / 10,  /* 0.1RPM → RPM */
                        ctrl_regs->acceleration,
                        (uint32_t)position,
                        (ctrl_regs->motion_type == MOTION_TYPE_ABSOLUTE),
                        sync_flag
                    );
                }
            }
            GATEWAY_DEBUG_PRINTF("电机%d位置运动: pos=%d\r\n", motor_id, (int)position);
            break;
            
        case MOTOR_CMD_VEL_MOVE:
            if (cb->motor_vel_control) {
                cb->motor_vel_control(
                    motor_id,
                    ctrl_regs->direction,
                    ctrl_regs->speed / 10,
                    ctrl_regs->acceleration,
                    sync_flag
                );
            }
            GATEWAY_DEBUG_PRINTF("电机%d速度运动: speed=%d\r\n", motor_id, ctrl_regs->speed);
            break;
            
        case MOTOR_CMD_STOP:
            if (cb->motor_stop) {
                cb->motor_stop(motor_id, sync_flag);
            }
            GATEWAY_DEBUG_PRINTF("电机%d立即停止\r\n", motor_id);
            break;
            
        case MOTOR_CMD_HOMING:
            /* TODO: 实现回零命令，需要读取回零参数 */
            GATEWAY_DEBUG_PRINTF("电机%d回零（未实现）\r\n", motor_id);
            break;
            
        case MOTOR_CMD_CLEAR_POS:
            if (cb->motor_reset_position) {
                cb->motor_reset_position(motor_id);
            }
            GATEWAY_DEBUG_PRINTF("电机%d清零位置\r\n", motor_id);
            break;
            
        case MOTOR_CMD_CLEAR_PROTECT:
            /* TODO: 实现解除保护命令 */
            GATEWAY_DEBUG_PRINTF("电机%d解除保护（未实现）\r\n", motor_id);
            break;
            
        default:
            return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    return 0;
}

/**
 * @brief  获取电机控制寄存器指针
 */
motor_control_regs_t* modbus_gateway_get_motor_control_regs(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > MODBUS_MAX_MOTORS) {
        return NULL;
    }
    return &g_motor_control_regs[motor_id - 1];
}

/**
 * @brief  获取电机状态寄存器指针
 */
motor_status_regs_t* modbus_gateway_get_motor_status_regs(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > MODBUS_MAX_MOTORS) {
        return NULL;
    }
    return &g_motor_status_regs[motor_id - 1];
}

/**
 * @brief  获取全局控制寄存器指针
 */
global_control_regs_t* modbus_gateway_get_global_regs(void)
{
    return &g_global_regs;
}

/* ======================== 私有函数实现 ======================== */

/**
 * @brief  根据地址读取寄存器值
 */
static uint16_t read_register_by_address(uint16_t addr)
{
    uint16_t *reg_ptr;
    uint8_t motor_id, offset;
    
    /* 全局控制区（0x0000-0x007F） */
    if (addr >= MODBUS_REG_GLOBAL_BASE && addr < MODBUS_REG_GLOBAL_BASE + 128) {
        reg_ptr = (uint16_t*)&g_global_regs;
        return reg_ptr[addr - MODBUS_REG_GLOBAL_BASE];
    }
    
    /* 电机控制区（0x0100-0x04FF） */
    if (addr >= MODBUS_REG_MOTOR_CTRL_BASE && addr < MODBUS_REG_MOTOR_CTRL_BASE + MODBUS_MAX_MOTORS * MODBUS_REGS_PER_MOTOR) {
        motor_id = (addr - MODBUS_REG_MOTOR_CTRL_BASE) / MODBUS_REGS_PER_MOTOR;
        offset = (addr - MODBUS_REG_MOTOR_CTRL_BASE) % MODBUS_REGS_PER_MOTOR;
        reg_ptr = (uint16_t*)&g_motor_control_regs[motor_id];
        return reg_ptr[offset];
    }
    
    /* 电机状态区（0x0500-0x08FF） */
    if (addr >= MODBUS_REG_MOTOR_STATUS_BASE && addr < MODBUS_REG_MOTOR_STATUS_BASE + MODBUS_MAX_MOTORS * MODBUS_REGS_PER_MOTOR) {
        motor_id = (addr - MODBUS_REG_MOTOR_STATUS_BASE) / MODBUS_REGS_PER_MOTOR;
        offset = (addr - MODBUS_REG_MOTOR_STATUS_BASE) % MODBUS_REGS_PER_MOTOR;
        reg_ptr = (uint16_t*)&g_motor_status_regs[motor_id];
        return reg_ptr[offset];
    }
    
    return 0;
}

/**
 * @brief  根据地址写入寄存器值
 */
static int write_register_by_address(uint16_t addr, uint16_t value)
{
    uint16_t *reg_ptr;
    uint8_t motor_id, offset;
    
    /* 全局控制区 */
    if (addr >= MODBUS_REG_GLOBAL_BASE && addr < MODBUS_REG_GLOBAL_BASE + 128) {
        reg_ptr = (uint16_t*)&g_global_regs;
        reg_ptr[addr - MODBUS_REG_GLOBAL_BASE] = value;
        
        /* 处理特殊寄存器 */
        if (addr == REG_SYS_DEVICE_ID) {
            modbus_rtu_set_slave_address((uint8_t)value);
        } else if (addr == REG_SYS_RESET && value == 0xAA55) {
            /* TODO: 系统复位 */
        }
        
        return 0;
    }
    
    /* 电机控制区 */
    if (addr >= MODBUS_REG_MOTOR_CTRL_BASE && addr < MODBUS_REG_MOTOR_CTRL_BASE + MODBUS_MAX_MOTORS * MODBUS_REGS_PER_MOTOR) {
        motor_id = (addr - MODBUS_REG_MOTOR_CTRL_BASE) / MODBUS_REGS_PER_MOTOR + 1;
        offset = (addr - MODBUS_REG_MOTOR_CTRL_BASE) % MODBUS_REGS_PER_MOTOR;
        reg_ptr = (uint16_t*)&g_motor_control_regs[motor_id - 1];
        reg_ptr[offset] = value;
        
        /* 检查是否写入EXEC_COMMAND寄存器 */
        if (offset == MOTOR_REG_EXEC_COMMAND && value != 0) {
            modbus_gateway_execute_motor_command(motor_id, (motor_command_t)value);
            /* 执行后清零命令寄存器 */
            reg_ptr[offset] = 0;
        }
        
        return 0;
    }
    
    /* 电机状态区（只读，拒绝写入） */
    if (addr >= MODBUS_REG_MOTOR_STATUS_BASE && addr < MODBUS_REG_MOTOR_STATUS_BASE + MODBUS_MAX_MOTORS * MODBUS_REGS_PER_MOTOR) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }
    
    return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
}

/**
 * @brief  检查地址是否可写
 */
static bool is_address_writable(uint16_t addr)
{
    /* 全局控制区（可写） */
    if (addr >= MODBUS_REG_GLOBAL_BASE && addr < MODBUS_REG_GLOBAL_BASE + 128) {
        /* 固件版本和硬件版本只读 */
        if (addr == REG_SYS_FIRMWARE_VERSION || addr == REG_SYS_HARDWARE_VERSION) {
            return false;
        }
        return true;
    }
    
    /* 电机控制区（可写） */
    if (addr >= MODBUS_REG_MOTOR_CTRL_BASE && addr < MODBUS_REG_MOTOR_CTRL_BASE + MODBUS_MAX_MOTORS * MODBUS_REGS_PER_MOTOR) {
        return true;
    }
    
    /* 电机状态区（只读） */
    if (addr >= MODBUS_REG_MOTOR_STATUS_BASE && addr < MODBUS_REG_MOTOR_STATUS_BASE + MODBUS_MAX_MOTORS * MODBUS_REGS_PER_MOTOR) {
        return false;
    }
    
    return false;
}


