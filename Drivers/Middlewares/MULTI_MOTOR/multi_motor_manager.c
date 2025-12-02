/**
 ******************************************************************************
 * @file    multi_motor_manager.c
 * @author  V3.5 Phase 2
 * @version V3.5
 * @date    2025-12-02
 * @brief   多电机管理器完整实现 (使用内存池)
 ******************************************************************************
 */

#include "multi_motor_manager.h"
#include "emm_v5.h"
#include "../../BSP/MEM_POOL/mem_pool.h"
#include "delay.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* 全局变量 */
static motor_manager_t g_mgr = {0};

/* 内部辅助函数 */
static int motor_find_index(uint8_t address)
{
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        if (g_mgr.motors[i] && g_mgr.motors[i]->address == address) {
            return i;
        }
    }
    return -1;
}

static void motor_update_health_success(motor_state_t *state)
{
    if (state->health < 100) state->health += 5;
    if (state->health > 100) state->health = 100;
    state->last_response_tick = HAL_GetTick();
    state->online = MOTOR_STATUS_ONLINE;
}

static void motor_update_health_failure(motor_state_t *state)
{
    if (state->health > 10) state->health -= 10;
    else state->health = 0;
    state->error_count++;
    state->last_error_tick = HAL_GetTick();
    if (state->health < MOTOR_HEALTH_THRESHOLD) state->online = MOTOR_STATUS_OFFLINE;
    else state->online = MOTOR_STATUS_ERROR;
}

/* 基础管理功能 */
HAL_StatusTypeDef motor_mgr_init(void)
{
    if (g_mgr.initialized) return HAL_OK;
    memset(&g_mgr, 0, sizeof(motor_manager_t));
    g_mgr.last_scan_tick = HAL_GetTick();
    g_mgr.last_update_tick = HAL_GetTick();
    g_mgr.initialized = true;
    printf("[MotorMgr] Initialized, max: %d\r\n", MAX_MOTOR_COUNT);
    return HAL_OK;
}

HAL_StatusTypeDef motor_mgr_add(uint8_t addr)
{
    if (addr == 0 || addr > 255) return HAL_ERROR;
    if (g_mgr.motor_count >= MAX_MOTOR_COUNT) return HAL_ERROR;
    if (motor_find_index(addr) >= 0) return HAL_ERROR;
    
    motor_state_t *state = (motor_state_t *)MEM_POOL_ALLOC_MOTOR_STATE();
    if (!state) {
        printf("[MotorMgr] Failed to alloc for motor %d\r\n", addr);
        return HAL_ERROR;
    }
    
    memset(state, 0, sizeof(motor_state_t));
    state->address = addr;
    state->health = 100;
    state->online = MOTOR_STATUS_OFFLINE;
    state->last_response_tick = HAL_GetTick();
    g_mgr.motors[g_mgr.motor_count++] = state;
    
    printf("[MotorMgr] Added motor %d (%d/%d)\r\n", addr, g_mgr.motor_count, MAX_MOTOR_COUNT);
    return HAL_OK;
}

HAL_StatusTypeDef motor_mgr_remove(uint8_t addr)
{
    int idx = motor_find_index(addr);
    if (idx < 0) return HAL_ERROR;
    
    mem_pool_free_motor_state(g_mgr.motors[idx]);
    for (int i = idx; i < g_mgr.motor_count - 1; i++) {
        g_mgr.motors[i] = g_mgr.motors[i + 1];
    }
    g_mgr.motors[--g_mgr.motor_count] = NULL;
    
    printf("[MotorMgr] Removed motor %d\r\n", addr);
    return HAL_OK;
}

motor_state_t* motor_mgr_find(uint8_t addr)
{
    int idx = motor_find_index(addr);
    return (idx >= 0) ? g_mgr.motors[idx] : NULL;
}

uint8_t motor_mgr_get_count(void)
{
    return g_mgr.motor_count;
}

/* 电机发现功能 */
uint8_t motor_mgr_scan(uint8_t start_addr, uint8_t end_addr)
{
    uint8_t found_count = 0;
    printf("[MotorMgr] Scanning %d-%d...\r\n", start_addr, end_addr);
    
    extern volatile uint8_t g_emm_frame_complete;
    extern uint8_t g_emm_rx_cmd[];
    extern uint16_t g_emm_rx_count;
    
    for (uint8_t addr = start_addr; addr <= end_addr; addr++) {
        Emm_V5_Read_Sys_Params(addr, 0x2A);
        uint32_t start_tick = HAL_GetTick();
        bool responded = false;
        
        while (HAL_GetTick() - start_tick < 50) {
            if (g_emm_frame_complete) {
                g_emm_frame_complete = 0;
                if (g_emm_rx_count >= 4 && g_emm_rx_cmd[1] == addr) {
                    responded = true;
                    break;
                }
            }
            delay_ms(1);
        }
        
        if (responded) {
            if (motor_find_index(addr) < 0) {
                if (motor_mgr_add(addr) == HAL_OK) found_count++;
            } else {
                motor_state_t *state = motor_mgr_find(addr);
                if (state) motor_update_health_success(state);
                found_count++;
            }
        }
    }
    
    g_mgr.last_scan_tick = HAL_GetTick();
    printf("[MotorMgr] Found %d motors\r\n", found_count);
    return found_count;
}
/* 状态查询功能 */
HAL_StatusTypeDef motor_mgr_query_enable(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    extern volatile uint8_t g_emm_frame_complete;
    extern uint8_t g_emm_rx_cmd[];
    extern uint16_t g_emm_rx_count;
    
    Emm_V5_Read_Sys_Params(address, 0x2A);
    state->total_commands++;
    uint32_t start_tick = HAL_GetTick();
    
    while (HAL_GetTick() - start_tick < 50) {
        if (g_emm_frame_complete) {
            g_emm_frame_complete = 0;
            if (g_emm_rx_count >= 6 && g_emm_rx_cmd[1] == address) {
                state->enabled = (g_emm_rx_cmd[4] == 1);
                motor_update_health_success(state);
                return HAL_OK;
            }
        }
        delay_ms(1);
    }
    
    motor_update_health_failure(state);
    return HAL_ERROR;
}

HAL_StatusTypeDef motor_mgr_query_position(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    extern volatile uint8_t g_emm_frame_complete;
    extern uint8_t g_emm_rx_cmd[];
    extern uint16_t g_emm_rx_count;
    
    Emm_V5_Read_Sys_Params(address, 0x33);
    state->total_commands++;
    uint32_t start_tick = HAL_GetTick();
    
    while (HAL_GetTick() - start_tick < 50) {
        if (g_emm_frame_complete) {
            g_emm_frame_complete = 0;
            if (g_emm_rx_count >= 8 && g_emm_rx_cmd[1] == address) {
                state->position = (int32_t)(g_emm_rx_cmd[4] | 
                                           (g_emm_rx_cmd[5] << 8) | 
                                           (g_emm_rx_cmd[6] << 16) | 
                                           (g_emm_rx_cmd[7] << 24));
                motor_update_health_success(state);
                return HAL_OK;
            }
        }
        delay_ms(1);
    }
    
    motor_update_health_failure(state);
    return HAL_ERROR;
}

HAL_StatusTypeDef motor_mgr_query_velocity(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    extern volatile uint8_t g_emm_frame_complete;
    extern uint8_t g_emm_rx_cmd[];
    extern uint16_t g_emm_rx_count;
    
    Emm_V5_Read_Sys_Params(address, 0x35);
    state->total_commands++;
    uint32_t start_tick = HAL_GetTick();
    
    while (HAL_GetTick() - start_tick < 50) {
        if (g_emm_frame_complete) {
            g_emm_frame_complete = 0;
            if (g_emm_rx_count >= 6 && g_emm_rx_cmd[1] == address) {
                state->velocity = (int16_t)(g_emm_rx_cmd[4] | (g_emm_rx_cmd[5] << 8));
                motor_update_health_success(state);
                return HAL_OK;
            }
        }
        delay_ms(1);
    }
    
    motor_update_health_failure(state);
    return HAL_ERROR;
}

HAL_StatusTypeDef motor_mgr_query_vbus(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    extern volatile uint8_t g_emm_frame_complete;
    extern uint8_t g_emm_rx_cmd[];
    extern uint16_t g_emm_rx_count;
    
    Emm_V5_Read_Sys_Params(address, 0x24);
    state->total_commands++;
    uint32_t start_tick = HAL_GetTick();
    
    while (HAL_GetTick() - start_tick < 50) {
        if (g_emm_frame_complete) {
            g_emm_frame_complete = 0;
            if (g_emm_rx_count >= 6 && g_emm_rx_cmd[1] == address) {
                state->vbus = (uint16_t)(g_emm_rx_cmd[4] | (g_emm_rx_cmd[5] << 8)) * 10;
                motor_update_health_success(state);
                return HAL_OK;
            }
        }
        delay_ms(1);
    }
    
    motor_update_health_failure(state);
    return HAL_ERROR;
}

HAL_StatusTypeDef motor_mgr_update_all_status(void)
{
    uint32_t now = HAL_GetTick();
    if (now - g_mgr.last_update_tick < MOTOR_STATUS_UPDATE_INTERVAL_MS) return HAL_OK;
    
    uint8_t success_count = 0;
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        motor_state_t *state = g_mgr.motors[i];
        if (!state || state->online == MOTOR_STATUS_OFFLINE) continue;
        
        if (motor_mgr_query_enable(state->address) == HAL_OK) success_count++;
        motor_mgr_query_position(state->address);
        motor_mgr_query_velocity(state->address);
        delay_ms(10);
    }
    
    g_mgr.last_update_tick = now;
    return (success_count > 0) ? HAL_OK : HAL_ERROR;
}

/* 单电机控制功能 */
HAL_StatusTypeDef motor_mgr_enable(uint8_t address, bool enable)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    Emm_V5_En_Control(address, enable, false);
    state->total_commands++;
    delay_ms(10);
    
    if (motor_mgr_query_enable(address) == HAL_OK) {
        printf("[MotorMgr] Motor %d %s\r\n", address, enable ? "enabled" : "disabled");
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef motor_mgr_move(uint8_t address, uint8_t direction, 
                                  uint16_t speed, uint8_t acceleration,
                                  uint32_t pulses, bool is_relative)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state || !state->enabled) return HAL_ERROR;
    
    state->direction = direction;
    state->target_speed = speed;
    state->acceleration = acceleration;
    state->is_moving = 1;
    
    Emm_V5_Pos_Control(address, direction, speed, acceleration, pulses, is_relative, false);
    state->total_commands++;
    
    printf("[MotorMgr] Motor %d move: dir=%d spd=%d acc=%d pulse=%lu\r\n",
           address, direction, speed, acceleration, pulses);
    return HAL_OK;
}

HAL_StatusTypeDef motor_mgr_stop(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    Emm_V5_Stop_Now(address, false);
    state->total_commands++;
    state->is_moving = 0;
    
    printf("[MotorMgr] Motor %d stopped\r\n", address);
    return HAL_OK;
}

HAL_StatusTypeDef motor_mgr_home(uint8_t address, uint8_t mode)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    Emm_V5_Origin_Trigger_Return(address, mode, false);
    state->total_commands++;
    
    printf("[MotorMgr] Motor %d homing (mode %d)\r\n", address, mode);
    return HAL_OK;
}

/* 批量控制功能 */
uint8_t motor_mgr_enable_all(void)
{
    uint8_t success_count = 0;
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        motor_state_t *state = g_mgr.motors[i];
        if (state && state->online != MOTOR_STATUS_OFFLINE) {
            if (motor_mgr_enable(state->address, true) == HAL_OK) success_count++;
            delay_ms(50);
        }
    }
    printf("[MotorMgr] Enabled %d/%d motors\r\n", success_count, g_mgr.motor_count);
    return success_count;
}

uint8_t motor_mgr_enable_batch(uint8_t *addrs, uint8_t count)
{
    uint8_t success_count = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (motor_mgr_enable(addrs[i], true) == HAL_OK) success_count++;
        delay_ms(50);
    }
    return success_count;
}

uint8_t motor_mgr_stop_all(void)
{
    uint8_t success_count = 0;
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        motor_state_t *state = g_mgr.motors[i];
        if (state) {
            motor_mgr_stop(state->address);
            success_count++;
        }
    }
    printf("[MotorMgr] Stopped %d motors\r\n", success_count);
    return success_count;
}

HAL_StatusTypeDef motor_mgr_sync_move(motor_move_t *moves, uint8_t count)
{
    if (!moves || count == 0) return HAL_ERROR;
    
    printf("[MotorMgr] Sync move: %d motors\r\n", count);
    
    for (uint8_t i = 0; i < count; i++) {
        motor_state_t *state = motor_mgr_find(moves[i].address);
        if (!state || !state->enabled) continue;
        
        Emm_V5_Pos_Control(moves[i].address, 
                          moves[i].direction,
                          moves[i].speed,
                          moves[i].acceleration,
                          moves[i].pulses,
                          moves[i].is_relative,
                          true);
        state->total_commands++;
        state->is_moving = 1;
        delay_ms(10);
    }
    
    Emm_V5_Synchronous_motion(0);
    printf("[MotorMgr] Sync move started\r\n");
    return HAL_OK;
}

/* 错误处理与监控 */
bool motor_mgr_is_timeout(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return true;
    return (HAL_GetTick() - state->last_response_tick) > MOTOR_RESPONSE_TIMEOUT_MS;
}

uint8_t motor_mgr_get_health(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    return state ? state->health : 0;
}

void motor_mgr_check_timeout(void)
{
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        motor_state_t *state = g_mgr.motors[i];
        if (!state) continue;
        
        if (motor_mgr_is_timeout(state->address)) {
            motor_update_health_failure(state);
            if (state->health < MOTOR_HEALTH_THRESHOLD) {
                printf("[MotorMgr] Motor %d offline (health=%d%%)\r\n", 
                       state->address, state->health);
            }
        }
    }
}

void motor_mgr_auto_recover(void)
{
    uint32_t now = HAL_GetTick();
    if (now - g_mgr.last_scan_tick < MOTOR_AUTO_RECOVER_INTERVAL_MS) return;
    
    printf("[MotorMgr] Auto recovery...\r\n");
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        motor_state_t *state = g_mgr.motors[i];
        if (!state || state->online != MOTOR_STATUS_OFFLINE) continue;
        
        if (motor_mgr_query_enable(state->address) == HAL_OK) {
            printf("[MotorMgr] Motor %d recovered\r\n", state->address);
            motor_mgr_enable(state->address, true);
        }
        delay_ms(100);
    }
    
    g_mgr.last_scan_tick = now;
}

HAL_StatusTypeDef motor_mgr_clear_errors(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) return HAL_ERROR;
    
    state->error_flags = MOTOR_ERROR_NONE;
    state->error_count = 0;
    printf("[MotorMgr] Cleared errors for motor %d\r\n", address);
    return HAL_OK;
}

/* 调试与诊断 */
void motor_mgr_print_status(uint8_t address)
{
    motor_state_t *state = motor_mgr_find(address);
    if (!state) {
        printf("[MotorMgr] Motor %d not found\r\n", address);
        return;
    }
    
    printf("=== Motor %d ===\r\n", address);
    printf("Online:   %s\r\n", state->online == MOTOR_STATUS_ONLINE ? "YES" : 
                               state->online == MOTOR_STATUS_ERROR ? "ERROR" : "OFFLINE");
    printf("Enabled:  %s\r\n", state->enabled ? "YES" : "NO");
    printf("Health:   %d%%\r\n", state->health);
    printf("Position: %ld\r\n", state->position);
    printf("Velocity: %d RPM\r\n", state->velocity);
    printf("Vbus:     %d mV\r\n", state->vbus);
    printf("Errors:   %lu\r\n", state->error_count);
    printf("Commands: %lu\r\n", state->total_commands);
    printf("Last Rsp: %lu ms ago\r\n", HAL_GetTick() - state->last_response_tick);
    printf("===============\r\n");
}

void motor_mgr_print_all_status(void)
{
    printf("\r\n=== Motor Manager ===\r\n");
    printf("Total: %d/%d\r\n", g_mgr.motor_count, MAX_MOTOR_COUNT);
    printf("====================\r\n\r\n");
    
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        motor_state_t *state = g_mgr.motors[i];
        if (state) {
            motor_mgr_print_status(state->address);
            printf("\r\n");
        }
    }
}

void motor_mgr_get_statistics(uint32_t *total_commands, uint32_t *total_errors)
{
    *total_commands = 0;
    *total_errors = 0;
    
    for (uint8_t i = 0; i < g_mgr.motor_count; i++) {
        motor_state_t *state = g_mgr.motors[i];
        if (state) {
            *total_commands += state->total_commands;
            *total_errors += state->error_count;
        }
    }
}

/* ========== V3.1兼容接口 (usmart_interface.c使用) ========== */
int multi_motor_scan(uint8_t start, uint8_t end)
{
    return motor_mgr_scan(start, end);
}

int multi_motor_map_address(uint8_t modbus_addr, uint8_t physical_addr)
{
    motor_state_t *state = motor_mgr_find(modbus_addr);
    if (state) {
        state->address = physical_addr;
        return 0;
    }
    return -1;
}

void multi_motor_print_list(void)
{
    motor_mgr_print_all_status();
}

void multi_motor_enable_batch(uint16_t mask, bool enable)
{
    for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (mask & (1 << i)) {
            motor_state_t *state = g_mgr.motors[i];
            if (state) motor_mgr_enable(state->address, enable);
        }
    }
}

void multi_motor_pos_control_batch(uint16_t mask, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (mask & (1 << i)) {
            motor_state_t *state = g_mgr.motors[i];
            if (state) motor_mgr_move(state->address, dir, speed, acc, pulses, false);
        }
    }
}

void multi_motor_vel_control_batch(uint16_t mask, uint8_t dir, uint16_t speed, uint8_t acc)
{
    for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (mask & (1 << i)) {
            motor_state_t *state = g_mgr.motors[i];
            if (state) Emm_V5_Vel_Control(state->address, dir, speed, acc, false);
        }
    }
}

void multi_motor_stop_batch(uint16_t mask)
{
    for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (mask & (1 << i)) {
            motor_state_t *state = g_mgr.motors[i];
            if (state) motor_mgr_stop(state->address);
        }
    }
}

void multi_motor_home_batch(uint16_t mask, uint8_t mode)
{
    for (uint8_t i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (mask & (1 << i)) {
            motor_state_t *state = g_mgr.motors[i];
            if (state) motor_mgr_home(state->address, mode);
        }
    }
}

void multi_motor_poll(void)
{
    motor_mgr_update_all_status();
    motor_mgr_check_timeout();
}
