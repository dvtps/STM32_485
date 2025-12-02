/**
 ******************************************************************************
 * @file    multi_motor_manager.h
 * @author  V3.5 Phase 2
 * @version V3.5
 * @date    2025-12-02
 * @brief   多电机管理器API接口 (基于内存池优化)
 ******************************************************************************
 */

#ifndef __MULTI_MOTOR_MANAGER_H
#define __MULTI_MOTOR_MANAGER_H

#include "motor_state.h"
#include "stm32f1xx_hal.h"

/* 基础管理功能 */
HAL_StatusTypeDef motor_mgr_init(void);
HAL_StatusTypeDef motor_mgr_add(uint8_t address);
HAL_StatusTypeDef motor_mgr_remove(uint8_t address);
motor_state_t* motor_mgr_find(uint8_t address);
uint8_t motor_mgr_get_count(void);

/* 电机发现功能 */
uint8_t motor_mgr_scan(uint8_t start_addr, uint8_t end_addr);

/* 状态查询功能 */
HAL_StatusTypeDef motor_mgr_query_enable(uint8_t address);
HAL_StatusTypeDef motor_mgr_query_position(uint8_t address);
HAL_StatusTypeDef motor_mgr_query_velocity(uint8_t address);
HAL_StatusTypeDef motor_mgr_query_vbus(uint8_t address);
HAL_StatusTypeDef motor_mgr_update_all_status(void);

/* 单电机控制功能 */
HAL_StatusTypeDef motor_mgr_enable(uint8_t address, bool enable);
HAL_StatusTypeDef motor_mgr_move(uint8_t address, uint8_t direction, 
                                  uint16_t speed, uint8_t acceleration,
                                  uint32_t pulses, bool is_relative);
HAL_StatusTypeDef motor_mgr_stop(uint8_t address);
HAL_StatusTypeDef motor_mgr_home(uint8_t address, uint8_t mode);

/* 批量控制功能 */
uint8_t motor_mgr_enable_all(void);
uint8_t motor_mgr_enable_batch(uint8_t *addrs, uint8_t count);
uint8_t motor_mgr_stop_all(void);
HAL_StatusTypeDef motor_mgr_sync_move(motor_move_t *moves, uint8_t count);

/* 错误处理与监控 */
bool motor_mgr_is_timeout(uint8_t address);
uint8_t motor_mgr_get_health(uint8_t address);
void motor_mgr_check_timeout(void);
void motor_mgr_auto_recover(void);
HAL_StatusTypeDef motor_mgr_clear_errors(uint8_t address);

/* 调试与诊断 */
void motor_mgr_print_status(uint8_t address);
void motor_mgr_print_all_status(void);
void motor_mgr_get_statistics(uint32_t *total_commands, uint32_t *total_errors);

#endif /* __MULTI_MOTOR_MANAGER_H */
