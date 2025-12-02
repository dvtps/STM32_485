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

#if FEATURE_MODBUS_ENABLE    /* 仅在启用Modbus功能时编译 */
#include "modbus_gateway.h"
#include "modbus_rtu.h"
#include "modbus_hal.h"
#include "error_handler.h"  /* V3.5 Phase 3: 参数验证宏 */
#include "emm_v5.h"          /* V3.5 Phase 5: 电机查询命令和响应解析 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>          /* V3.5 Phase 6: abs()函数 */
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

/* ======================== V3.5 Phase 4: 电机运行时状态缓存 ======================== */

/**
 * @brief 电机运行时状态结构体（用于状态轮询缓存）
 */
typedef struct {
    uint8_t  enabled;            /* 使能状态（0=未使能, 1=已使能） */
    uint8_t  ready;              /* 就绪状态（0=未就绪, 1=就绪） */
    int16_t  current_speed;      /* 当前速度(RPM, 有符号) */
    int32_t  current_position;   /* 当前位置(脉冲数, 有符号) */
    int32_t  target_position;    /* 目标位置(脉冲数, 有符号) V3.5 Phase 6 */
    uint16_t fault_flags;        /* 故障标志（bit0=堵转, bit1=过流, bit2=欠压等） */
    uint16_t fault_code;         /* 故障码（motor_fault_code_t, V3.5 Phase 6） */
    uint32_t last_query_tick;    /* 上次查询时间戳(ms) */
    uint8_t  query_pending;      /* 查询挂起标志（0=空闲, 1=等待响应） */
    uint8_t  timeout_count;      /* 连续超时计数（V3.5 Phase 6） */
    uint16_t fault_history;      /* 历史故障记录（V3.5 Phase 6，最近16次） */
} motor_runtime_state_t;

/* 电机运行时状态数组（最多支持8个电机） */
static motor_runtime_state_t g_motor_runtime_state[MODBUS_MAX_MOTORS] = {0};

/* V3.5 Phase 6: 电机查询统计数组（每个电机独立统计） */
static motor_query_stats_t g_motor_query_stats[MODBUS_MAX_MOTORS] = {0};

/* 状态轮询配置参数 */
#define MOTOR_QUERY_INTERVAL_MS     100     /* 每个电机查询间隔100ms */
#define MOTOR_QUERY_TIMEOUT_MS      50      /* 查询超时时间50ms */

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
 * @brief  周期性更新电机状态（主循环调用）
 * @note   V3.5 Phase 5: 完整实现电机状态轮询机制
 *         采用轮询策略：每次调用轮询1个电机，100ms间隔
 *         发送S_FLAG查询命令，获取使能/到位/堵转状态
 */
void modbus_gateway_update_motor_status(void)
{
    static uint8_t poll_motor_index = 0;  /* 当前轮询电机索引(0-7) */
    static uint32_t last_poll_tick = 0;   /* 上次轮询时间戳 */
    
    uint32_t current_tick = HAL_GetTick();
    motor_runtime_state_t *state;
    uint8_t motor_addr;
    
    /* 检查是否到达轮询间隔 */
    if ((current_tick - last_poll_tick) < MOTOR_QUERY_INTERVAL_MS) {
        return;  /* 未到时间，直接返回 */
    }
    
    last_poll_tick = current_tick;
    
    /* 获取当前电机地址（1-8） */
    motor_addr = poll_motor_index + 1;
    state = &g_motor_runtime_state[poll_motor_index];
    
    /* 检查是否有未完成的查询 */
    if (state->query_pending) {
        /* 超时处理 */
        if ((current_tick - state->last_query_tick) > MOTOR_QUERY_TIMEOUT_MS) {
            state->query_pending = 0;  /* 清除挂起标志 */
            state->timeout_count++;     /* V3.5 Phase 6: 增加超时计数 */
            g_motor_query_stats[poll_motor_index].timeouts++;  /* V3.5 Phase 6: 统计超时 */
            GATEWAY_DEBUG_PRINTF("电机%d查询超时（连续%d次）\r\n", motor_addr, state->timeout_count);
            
            /* V3.5 Phase 6: 连续3次超时判定为通信故障 */
            if (state->timeout_count >= MOTOR_FAULT_TIMEOUT_THRESHOLD) {
                state->fault_code |= MOTOR_FAULT_COMM_TIMEOUT;
                state->fault_flags |= 0x0002;  /* 设置通信超时标志位 */
                g_motor_status_regs[poll_motor_index].fault_code = state->fault_code;
                g_motor_status_regs[poll_motor_index].error_code = 0x04;  /* Modbus从站设备故障 */
            }
        } else {
            /* 仍在等待响应，跳过本次轮询 */
            poll_motor_index = (poll_motor_index + 1) % MODBUS_MAX_MOTORS;
            return;
        }
    }
    
    /* V3.5 Phase 5: 发送状态查询命令（查询S_FLAG状态标志位） */
    Emm_V5_Read_Sys_Params(motor_addr, S_FLAG);
    
    state->last_query_tick = current_tick;
    state->query_pending = 1;  /* 设置挂起标志，等待响应解析 */
    
    /* V3.5 Phase 6: 统计查询次数 */
    g_motor_query_stats[poll_motor_index].total_queries++;
    
    /* 切换到下一个电机 */
    poll_motor_index = (poll_motor_index + 1) % MODBUS_MAX_MOTORS;
}

/**
 * @brief  处理电机查询响应（V3.5 Phase 5新增）
 * @param  motor_addr: 电机地址（1-8）
 * @param  data: 响应帧数据
 * @param  len: 响应帧长度
 * @retval None
 * @note   在USART2 IDLE中断中调用，更新g_motor_runtime_state
 */
void modbus_gateway_handle_motor_response(uint8_t motor_addr, const uint8_t *data, uint16_t len)
{
    emm_response_t response;
    motor_runtime_state_t *state;
    
    /* 参数验证 */
    if (motor_addr < 1 || motor_addr > MODBUS_MAX_MOTORS || data == NULL || len == 0) {
        return;
    }
    
    /* 解析响应帧 */
    if (!Emm_V5_Parse_Response(data, len, &response)) {
        return;  /* 解析失败 */
    }
    
    /* 更新状态缓存 */
    state = &g_motor_runtime_state[motor_addr - 1];
    state->query_pending = 0;  /* 清除查询挂起标志 */
    
    /* V3.5 Phase 6: 成功响应，清除超时计数 */
    if (state->timeout_count > 0) {
        state->timeout_count = 0;  /* 重置超时计数 */
        /* 清除通信超时故障标志（如果有） */
        state->fault_code &= ~MOTOR_FAULT_COMM_TIMEOUT;
        state->fault_flags &= ~0x0002;
    }
    
    /* V3.5 Phase 6: 统计成功响应 */
    g_motor_query_stats[motor_addr - 1].successful_responses++;
    
    /* V3.5 Phase 6: 计算成功率 */
    if (g_motor_query_stats[motor_addr - 1].total_queries > 0) {
        g_motor_query_stats[motor_addr - 1].success_rate = 
            (float)g_motor_query_stats[motor_addr - 1].successful_responses * 100.0f / 
            (float)g_motor_query_stats[motor_addr - 1].total_queries;
    }
    
    /* 根据功能码更新对应状态 */
    switch (response.cmd) {
        case 0x3A:  /* S_FLAG - 状态标志位 */
            state->enabled = response.data.flags.enabled;
            state->ready = response.data.flags.arrived;  /* 到位标志表示就绪 */
            state->fault_flags = response.data.flags.stalled ? 0x0001 : 0x0000;  /* bit0=堵转 */
            
            /* 同步到Modbus离散输入 */
            if (response.data.flags.arrived) {
                uint8_t byte_idx = (motor_addr - 1) / 8;
                uint8_t bit_idx = (motor_addr - 1) % 8;
                g_discrete_inputs.inputs[byte_idx] |= (1 << bit_idx);
            }
            break;
            
        case 0x35:  /* S_VEL - 速度 */
            state->current_speed = response.data.velocity;
            g_motor_status_regs[motor_addr - 1].real_speed = (uint16_t)response.data.velocity;
            break;
            
        case 0x36:  /* S_CPOS - 位置 */
            state->current_position = response.data.position;
            g_motor_status_regs[motor_addr - 1].real_position_h = (uint16_t)(response.data.position >> 16);
            g_motor_status_regs[motor_addr - 1].real_position_l = (uint16_t)(response.data.position & 0xFFFF);
            break;
            
        default:
            break;
    }
}

/**
 * @brief  周期性检查电机故障状态（V3.5 Phase 6新增）
 * @note   在主循环中调用，建议250ms周期
 *         检测项：通信超时、堵转、超速、位置误差、使能失败
 */
void modbus_gateway_check_motor_faults(void)
{
    static uint32_t last_check_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    
    /* 250ms周期检测 */
    if ((current_tick - last_check_tick) < MOTOR_FAULT_CHECK_INTERVAL_MS) {
        return;
    }
    last_check_tick = current_tick;
    
    /* 遍历所有电机 */
    for (uint8_t i = 0; i < MODBUS_MAX_MOTORS; i++) {
        motor_runtime_state_t *state = &g_motor_runtime_state[i];
        uint16_t old_fault_code = state->fault_code;
        
        /* 1. 通信完全丢失检测：5秒无响应 */
        if ((current_tick - state->last_query_tick) > 5000 && state->last_query_tick > 0) {
            state->fault_code |= MOTOR_FAULT_COMM_LOST;
            state->fault_flags |= 0x0020;
        } else {
            state->fault_code &= ~MOTOR_FAULT_COMM_LOST;
            state->fault_flags &= ~0x0020;
        }
        
        /* 2. 堵转检测：从EMM_V5响应的stalled标志已在handle_motor_response中设置 */
        /* 此处仅确保状态一致性 */
        if (state->fault_flags & 0x0001) {
            state->fault_code |= MOTOR_FAULT_STALLED;
        }
        
        /* 3. 超速检测：实际速度 > 5000 RPM */
        if (abs(state->current_speed) > MOTOR_FAULT_SPEED_MAX) {
            state->fault_code |= MOTOR_FAULT_OVERSPEED;
            state->fault_flags |= 0x0004;
        } else {
            state->fault_code &= ~MOTOR_FAULT_OVERSPEED;
            state->fault_flags &= ~0x0004;
        }
        
        /* 4. 位置误差检测：|目标-实际| > 500脉冲 */
        if (state->target_position != 0) {  /* 仅在有目标位置时检测 */
            int32_t position_error = abs(state->target_position - state->current_position);
            if (position_error > MOTOR_FAULT_POSITION_ERROR_MAX) {
                state->fault_code |= MOTOR_FAULT_POSITION_ERR;
                state->fault_flags |= 0x0008;
            } else {
                state->fault_code &= ~MOTOR_FAULT_POSITION_ERR;
                state->fault_flags &= ~0x0008;
            }
        }
        
        /* 5. 使能失败检测：control_regs.enable=1但state.enabled=0 */
        if (g_motor_control_regs[i].enable == 1 && state->enabled == 0) {
            /* 需要一定延迟判断（500ms），避免误判 */
            static uint32_t enable_fail_tick[MODBUS_MAX_MOTORS] = {0};
            if (enable_fail_tick[i] == 0) {
                enable_fail_tick[i] = current_tick;
            } else if ((current_tick - enable_fail_tick[i]) > 500) {
                state->fault_code |= MOTOR_FAULT_ENABLE_FAIL;
                state->fault_flags |= 0x0010;
            }
        } else {
            state->fault_code &= ~MOTOR_FAULT_ENABLE_FAIL;
            state->fault_flags &= ~0x0010;
        }
        
        /* 6. 同步到Modbus状态寄存器 */
        if (state->fault_code != old_fault_code) {
            g_motor_status_regs[i].fault_code = state->fault_code;
            
            /* 设置历史故障记录（最近16次，循环移位） */
            if (state->fault_code != MOTOR_FAULT_NONE) {
                state->fault_history = (state->fault_history << 1) | 0x0001;
            } else {
                state->fault_history = (state->fault_history << 1);
            }
            
            GATEWAY_DEBUG_PRINTF("电机%d故障码更新: 0x%04X\r\n", i+1, state->fault_code);
        }
    }
}

/**
 * @brief  获取指定电机的查询统计信息（V3.5 Phase 6新增）
 * @param  motor_index: 电机索引（0-7）
 * @retval 查询统计结构体
 */
motor_query_stats_t modbus_gateway_get_query_stats(uint8_t motor_index)
{
    motor_query_stats_t empty_stats = {0};
    
    if (motor_index >= MODBUS_MAX_MOTORS) {
        return empty_stats;
    }
    
    return g_motor_query_stats[motor_index];
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
            /* V3.5 Phase 4: 实现回零命令 */
            if (cb->motor_home) {
                /* 读取回零参数（默认值：模式0，速度300RPM） */
                uint8_t home_mode = (ctrl_regs->home_mode > 3) ? 0 : (uint8_t)ctrl_regs->home_mode;
                cb->motor_home(motor_id, home_mode, sync_flag);
                GATEWAY_DEBUG_PRINTF("电机%d回零: mode=%d\r\n", motor_id, home_mode);
            }
            break;
            
        case MOTOR_CMD_CLEAR_POS:
            if (cb->motor_reset_position) {
                cb->motor_reset_position(motor_id);
            }
            GATEWAY_DEBUG_PRINTF("电机%d清零位置\r\n", motor_id);
            break;
            
        case MOTOR_CMD_CLEAR_PROTECT:
            /* V3.5 Phase 4: 实现解除保护命令 */
            if (cb->motor_release) {
                cb->motor_release(motor_id);
                GATEWAY_DEBUG_PRINTF("电机%d解除保护\r\n", motor_id);
            }
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

/**
 * @brief  检查电机是否使能（V3.5 Phase 4新增）
 */
uint8_t modbus_gateway_is_motor_enabled(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > MODBUS_MAX_MOTORS) {
        return 0;
    }
    return g_motor_runtime_state[motor_id - 1].enabled;
}

/**
 * @brief  检查电机是否就绪（V3.5 Phase 4新增）
 */
uint8_t modbus_gateway_is_motor_ready(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > MODBUS_MAX_MOTORS) {
        return 0;
    }
    return g_motor_runtime_state[motor_id - 1].ready;
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
    if (addr < MODBUS_REG_GLOBAL_BASE + 128) {
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
    if (addr < MODBUS_REG_GLOBAL_BASE + 128) {
        reg_ptr = (uint16_t*)&g_global_regs;
        reg_ptr[addr - MODBUS_REG_GLOBAL_BASE] = value;
        
        /* 处理特殊寄存器 */
        if (addr == REG_SYS_DEVICE_ID) {
            modbus_rtu_set_slave_address((uint8_t)value);
        } else if (addr == REG_SYS_RESET && value == 0xAA55) {
            /* V3.5 Phase 4: 系统复位（魔术值0xAA55触发） */
            GATEWAY_DEBUG_PRINTF("系统复位触发...\r\n");
            NVIC_SystemReset();  /* 软件复位 */
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
    if (addr < MODBUS_REG_GLOBAL_BASE + 128) {
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

#endif /* FEATURE_MODBUS_ENABLE */
