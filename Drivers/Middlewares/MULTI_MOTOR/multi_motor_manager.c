/**
 ******************************************************************************
 * @file    multi_motor_manager.c
 * @author  STM32_485 Project (Middlewares Layer)
 * @version V3.1
 * @date    2025-12-02
 * @brief   多电机管理中间件实现
 ******************************************************************************
 * @attention
 * 
 * 架构依赖关系：
 * - 向上提供：标准化多电机管理API（供App层调用）
 * - 向下依赖：BSP/EMM_V5层的Emm_V5_*函数
 * - 平级依赖：SYSTEM/delay层的延时函数
 * 
 * 可移植性：
 * - 只需替换Emm_V5_*函数调用，即可适配其他电机驱动
 * - 不包含App层业务逻辑（如按键处理、UI显示等）
 * 
 ******************************************************************************
 */

#include "multi_motor_manager.h"
#include "emm_v5.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>

/* 全局电机列表 */
static motor_info_t g_motor_list[MULTI_MOTOR_MAX_COUNT] = {0};
static uint8_t g_motor_online_count = 0;
static uint32_t g_last_poll_time = 0;

/**
 * @brief       多电机管理器初始化
 */
int multi_motor_init(void)
{
    memset(g_motor_list, 0, sizeof(g_motor_list));
    g_motor_online_count = 0;
    
    /* 初始化默认地址映射（1-16号电机对应物理地址1-16） */
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        g_motor_list[i].modbus_addr = i + 1;
        g_motor_list[i].physical_addr = i + 1;
        g_motor_list[i].is_online = false;
        g_motor_list[i].state = MOTOR_STATE_OFFLINE;
        g_motor_list[i].max_speed = 500;
        g_motor_list[i].acceleration = 10;
    }
    
    printf("[MultiMotor] Manager initialized\r\n");
    return 0;
}

/**
 * @brief       扫描RS485总线，发现在线电机
 */
int multi_motor_scan(uint8_t start_addr, uint8_t end_addr)
{
    uint8_t found = 0;
    
    printf("[MultiMotor] Scanning bus (addr %d-%d)...\r\n", start_addr, end_addr);
    
    for (uint8_t addr = start_addr; addr <= end_addr && addr <= 16; addr++) {
        /* 发送查询命令（读取电机使能状态） */
        Emm_V5_Read_Sys_Params(addr, false);
        delay_ms(50);  /* 等待响应 */
        
        /* 检查是否有响应（通过检查g_emm_frame_complete标志） */
        extern volatile uint8_t g_emm_frame_complete;
        if (g_emm_frame_complete) {
            g_emm_frame_complete = 0;
            
            /* 标记电机在线 */
            if (addr <= MULTI_MOTOR_MAX_COUNT) {
                motor_info_t *motor = &g_motor_list[addr - 1];
                motor->is_online = true;
                motor->state = MOTOR_STATE_ONLINE;
                motor->physical_addr = addr;
                motor->modbus_addr = addr;
                found++;
                
                printf("[MultiMotor] Found motor at addr=%d\r\n", addr);
            }
        }
    }
    
    g_motor_online_count = found;
    printf("[MultiMotor] Scan complete: %d motors online\r\n", found);
    return found;
}

/**
 * @brief       设置电机地址映射
 */
int multi_motor_map_address(uint8_t modbus_addr, uint8_t physical_addr)
{
    if (modbus_addr == 0 || modbus_addr > MULTI_MOTOR_MAX_COUNT) {
        return -1;
    }
    
    motor_info_t *motor = &g_motor_list[modbus_addr - 1];
    motor->modbus_addr = modbus_addr;
    motor->physical_addr = physical_addr;
    
    printf("[MultiMotor] Map: Modbus#%d -> Physical#%d\r\n", modbus_addr, physical_addr);
    return 0;
}

/**
 * @brief       获取电机信息
 */
motor_info_t* multi_motor_get_info(uint8_t modbus_addr)
{
    if (modbus_addr == 0 || modbus_addr > MULTI_MOTOR_MAX_COUNT) {
        return NULL;
    }
    return &g_motor_list[modbus_addr - 1];
}

/**
 * @brief       获取在线电机数量
 */
uint8_t multi_motor_get_online_count(void)
{
    return g_motor_online_count;
}

/* ======================== 批量控制实现 ======================== */

#if FEATURE_MULTI_MOTOR_BATCH_ENABLE

/**
 * @brief       批量使能电机
 */
int multi_motor_enable_batch(uint16_t motor_mask, bool enable)
{
    int success = 0;
    
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        if (motor_mask & (1 << i)) {
            motor_info_t *motor = &g_motor_list[i];
            if (motor->is_online) {
                Emm_V5_En_Control(motor->physical_addr, enable, true);
                motor->enabled = enable;
                motor->tx_count++;
                success++;
            }
        }
    }
    
    /* 触发同步执行 */
    if (success > 0) {
        Emm_V5_Synchronous_motion(0);
    }
    
    return 0;
}

/**
 * @brief       批量位置控制（同步运动）
 */
int multi_motor_pos_control_batch(uint16_t motor_mask, uint8_t dir, 
                                   uint16_t speed, uint8_t acc, uint32_t pulses)
{
    int success = 0;
    
    /* 第一阶段：发送同步命令（snF=true） */
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        if (motor_mask & (1 << i)) {
            motor_info_t *motor = &g_motor_list[i];
            if (motor->is_online && motor->enabled) {
                Emm_V5_Pos_Control(motor->physical_addr, dir, speed, acc, pulses, false, true);
                motor->state = MOTOR_STATE_RUNNING;
                motor->tx_count++;
                success++;
            }
        }
    }
    
    /* 第二阶段：触发同步执行 */
    if (success > 0) {
        Emm_V5_Synchronous_motion(0);
        printf("[MultiMotor] Batch pos control: %d motors, dir=%d, speed=%d, pulses=%lu\r\n",
               success, dir, speed, pulses);
    }
    
    return success > 0 ? 0 : -1;
}

/**
 * @brief       批量速度控制
 */
int multi_motor_vel_control_batch(uint16_t motor_mask, uint8_t dir, 
                                   uint16_t speed, uint8_t acc)
{
    int success = 0;
    
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        if (motor_mask & (1 << i)) {
            motor_info_t *motor = &g_motor_list[i];
            if (motor->is_online && motor->enabled) {
                Emm_V5_Vel_Control(motor->physical_addr, dir, speed, acc, true);
                motor->state = MOTOR_STATE_RUNNING;
                motor->tx_count++;
                success++;
            }
        }
    }
    
    if (success > 0) {
        Emm_V5_Synchronous_motion(0);
        printf("[MultiMotor] Batch vel control: %d motors, speed=%d RPM\r\n", success, speed);
    }
    
    return success > 0 ? 0 : -1;
}

/**
 * @brief       批量急停
 */
int multi_motor_stop_batch(uint16_t motor_mask)
{
    int success = 0;
    
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        if (motor_mask & (1 << i)) {
            motor_info_t *motor = &g_motor_list[i];
            if (motor->is_online) {
                Emm_V5_Stop_Now(motor->physical_addr, false);
                motor->state = MOTOR_STATE_ONLINE;
                motor->tx_count++;
                success++;
            }
        }
    }
    
    printf("[MultiMotor] Batch stop: %d motors\r\n", success);
    return success > 0 ? 0 : -1;
}

/**
 * @brief       批量回零
 */
int multi_motor_home_batch(uint16_t motor_mask, uint8_t mode)
{
    int success = 0;
    
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        if (motor_mask & (1 << i)) {
            motor_info_t *motor = &g_motor_list[i];
            if (motor->is_online) {
                Emm_V5_Origin_Trigger_Return(motor->physical_addr, mode, true);
                motor->state = MOTOR_STATE_RUNNING;
                motor->tx_count++;
                success++;
            }
        }
    }
    
    if (success > 0) {
        Emm_V5_Synchronous_motion(0);
        printf("[MultiMotor] Batch home: %d motors, mode=%d\r\n", success, mode);
    }
    
    return success > 0 ? 0 : -1;
}

#else  /* !FEATURE_MULTI_MOTOR_BATCH_ENABLE */

/* 批处理函数禁用时提供占位实现 */
int multi_motor_enable_batch(uint16_t motor_mask, bool enable) 
{ 
    (void)motor_mask; (void)enable;
    return -1; 
}

int multi_motor_pos_control_batch(uint16_t motor_mask, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses) 
{ 
    (void)motor_mask; (void)dir; (void)speed; (void)acc; (void)pulses;
    return -1; 
}

int multi_motor_vel_control_batch(uint16_t motor_mask, uint8_t dir, uint16_t speed, uint8_t acc) 
{ 
    (void)motor_mask; (void)dir; (void)speed; (void)acc;
    return -1; 
}

int multi_motor_stop_batch(uint16_t motor_mask) 
{ 
    (void)motor_mask;
    return -1; 
}

int multi_motor_home_batch(uint16_t motor_mask, uint8_t mode) 
{ 
    (void)motor_mask; (void)mode;
    return -1; 
}

#endif  /* FEATURE_MULTI_MOTOR_BATCH_ENABLE */

/* ======================== 状态轮询实现 ======================== */

/**
 * @brief       更新所有在线电机状态（简化版，实际需要读取电机响应）
 */
int multi_motor_update_status(void)
{
    uint32_t current_time = HAL_GetTick();
    
    /* 1秒轮询一次 */
    if (current_time - g_last_poll_time < MULTI_MOTOR_STATUS_POLL_MS) {
        return 0;
    }
    g_last_poll_time = current_time;
    
    int updated = 0;
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        motor_info_t *motor = &g_motor_list[i];
        if (motor->is_online) {
            /* TODO: 实际实现需要发送查询命令并解析响应 */
            /* Emm_V5_Read_Sys_Params(motor->physical_addr, false); */
            updated++;
        }
    }
    
    return updated;
}

/**
 * @brief       打印电机列表
 */
void multi_motor_print_list(void)
{
    printf("\r\n========== Motor List ==========\r\n");
    printf("Modbus# | Phys# | State   | Pos    | Speed\r\n");
    printf("--------|-------|---------|--------|-------\r\n");
    
    for (uint8_t i = 0; i < MULTI_MOTOR_MAX_COUNT; i++) {
        motor_info_t *motor = &g_motor_list[i];
        if (motor->is_online) {
            const char *state_str[] = {"OFFLINE", "ONLINE", "RUNNING", "ERROR"};
            printf("   %2d   |   %2d  | %-7s | %6ld | %5d\r\n",
                   motor->modbus_addr,
                   motor->physical_addr,
                   state_str[motor->state],
                   (long)motor->current_position,
                   motor->current_speed);
        }
    }
    printf("Total: %d motors online\r\n", g_motor_online_count);
    printf("================================\r\n\r\n");
}
