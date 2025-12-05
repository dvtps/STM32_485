void printer_clear_emergency_stop(void); /* 解除急停状态 */
/**
 ******************************************************************************
 * @file    usmart_interface.h
 * @author  STM32_485 Project
 * @version V3.1
 * @date    2025-12-01
 * @brief   USMART可调用函数接口声明（扩展版）
 ******************************************************************************
 */

#ifndef __USMART_INTERFACE_H
#define __USMART_INTERFACE_H

#include <stdint.h>

/* ============ 原有接口（保持兼容） ============ */

/* 内存读写 */
uint32_t read_addr(uint32_t addr);
void write_addr(uint32_t addr, uint32_t val);

/* LED控制 */
void led_init(void);

/* 按键控制 */
void key_init(void);
uint8_t key_scan(uint8_t mode);

/* 看门狗 */
void iwdg_init(uint8_t prer, uint16_t rlr);
void iwdg_feed(void);

/* 单电机控制（保留有调试输出的函数） */
void motor_enable(uint8_t addr, uint8_t enable);

/* 注意：其他电机控制函数已直接注册底层 EMM_V5 API，无套娃层 */
/* 使用方法：
 * Emm_V5_Pos_Control(addr,dir,speed,acc,pulses,0,0)  - 位置控制
 * Emm_V5_Vel_Control(addr,dir,speed,acc,0)           - 速度控制
 * Emm_V5_Stop_Now(addr,0)                            - 立即停止
 * Emm_V5_Origin_Trigger_Return(addr,mode,0)          - 回零
 * Emm_V5_Read_Sys_Params(addr,0)                     - 读取状态
 */

/* ============ V3.1新增：多电机管理接口 ============ */

/* 电机扫描与映射 */
void multi_scan(uint8_t start, uint8_t end);          /* 扫描电机 */
void multi_map(uint8_t modbus, uint8_t physical);     /* 地址映射 */
void multi_list(void);                                 /* 列出电机 */

/* 批量控制 */
void multi_enable(uint16_t mask, uint8_t enable);     /* 批量使能 */
void multi_pos(uint16_t mask, uint8_t dir, uint16_t speed, uint32_t pulses);  /* 批量位置 */
void multi_vel(uint16_t mask, uint8_t dir, uint16_t speed);                   /* 批量速度 */
void multi_stop(uint16_t mask);                        /* 批量停止 */
void multi_home(uint16_t mask, uint8_t mode);         /* 批量回零 */

/* ============ V3.1新增：协议统计接口 ============ */

void proto_stats(void);                                /* 显示协议统计 */
void proto_reset(void);                                /* 重置统计 */

/* ============ V3.5 Phase 8 P1: 增量CRC调试接口 ============ */

void crc_stats(void);                                  /* CRC统计信息 */
void fifo_stats(void);                                 /* FIFO统计信息 */

/* ============ V3.7: 电机监控系统 ============ */

/* ============ V3.5 Phase 1: 内存池调试接口 ============ */

void mem_stats(void);                                  /* 内存池统计信息 */
void mem_check_leaks(void);                            /* 检查内存泄漏 */
void mem_reset_stats(uint8_t type);                    /* 重置统计计数器 */

/* ============ V3.5 Phase 3: 多电机管理器调试接口 ============ */

void mgr_scan(uint8_t start, uint8_t end);            /* 扫描电机（motor_mgr版本） */
void mgr_list(void);                                   /* 列出所有电机状态 */
void mgr_info(uint8_t addr);                          /* 显示单个电机详细信息 */
void mgr_enable(uint8_t addr, uint8_t enable);        /* 使能/失能电机 */
void mgr_move(uint8_t addr, uint8_t dir, uint16_t speed, uint32_t pulses);  /* 位置控制 */
void mgr_stop(uint8_t addr);                          /* 急停电机 */
void mgr_stop_all(void);                              /* 全体急停 */
void mgr_health(uint8_t addr);                        /* 查询健康度 */
void mgr_recover(void);                               /* 手动触发自动恢复 */
void mgr_query_pos(uint8_t addr);                     /* 查询位置 */
void mgr_query_vel(uint8_t addr);                     /* 查询速度 */
void mgr_query_vbus(uint8_t addr);                    /* 查询电压 */

/* 硬件测试函数 */
void mem_test_stress(uint16_t count);                  /* 压力测试：循环分配释放 */
void mem_test_leak(uint8_t block_count);               /* 泄漏测试：故意不释放 */
void mem_test_concurrent(void);                        /* 并发测试：分配至池满 */

/* RS485硬件测试 */
void rs485_loopback_test(void);                        /* RS485回环测试：验证收发通路 */
void rs485_debug_status(void);                         /* RS485调试：显示中断/FIFO状态 */
void rs485_rxne_test(void);                            /* RXNE中断测试：单字节收发 */
void rs485_nvic_test(void);                            /* NVIC配置检查 */
void rs485_polling_test(void);                         /* 轮询模式测试：不依赖中断 */
void rs485_motor_response_test(void);                  /* 电机响应测试：检查IDLE中断 */

/* ============ 3D打印机控制接口 ============ */

/* 电机使能控制 */
void printer_enable_all(void);                         /* 使能所有4台电机 */
void printer_disable_all(void);                        /* 失能所有4台电机 */

/* 单轴运动控制（脉冲数单位，底层调试用） */
void printer_move_x(int32_t distance, uint16_t speed); /* X轴移动（脉冲数） */
void printer_move_y(int32_t distance, uint16_t speed); /* Y轴移动（脉冲数，双电机同步） */
void printer_move_z(int32_t distance, uint16_t speed); /* Z轴移动（脉冲数） */

/* 多轴协同运动（脉冲数单位） */
void printer_move_xyz(int32_t x, int32_t y, int32_t z, uint16_t speed);  /* XYZ三轴协同运动 */

/* 单轴运动控制（毫米单位，应用层推荐） */
void printer_move_x_mm(float distance_mm, uint16_t speed); /* X轴移动（mm） */
void printer_move_y_mm(float distance_mm, uint16_t speed); /* Y轴移动（mm，双电机同步） */
void printer_move_z_mm(float distance_mm, uint16_t speed); /* Z轴移动（mm） */

/* 多轴协同运动（毫米单位）*/
void printer_xyz_mm(float x_mm, float y_mm, float z_mm, uint16_t speed);  /* XYZ三轴协同运动（mm）*/

/* 整数版本mm移动函数（USMART兼容，精度0.02mm）*/
void printer_move_x_mm_int(int16_t distance_50um, uint16_t speed); /* X轴移动（50微米：50=1.0mm）*/
void printer_move_y_mm_int(int16_t distance_50um, uint16_t speed); /* Y轴移动（50微米）*/
void printer_move_z_mm_int(int16_t distance_50um, uint16_t speed); /* Z轴移动（50微米）*/
void printer_xyz_mm_int(int16_t x_50um, int16_t y_50um, int16_t z_50um, uint16_t speed); /* XYZ同步（50微米）*/

/* 回零功能 */
void printer_home_x(void);                             /* X轴回零 */
void printer_home_y(void);                             /* Y轴回零（双电机同步） */
void printer_home_z(void);                             /* Z轴回零 */
void printer_home_all_axes(void);                      /* 全轴回零（Z→Y→X顺序） */

/* 紧急停止 */
void printer_estop(void);                              /* 紧急停止所有电机 */

/* 状态查询 */
void printer_show_status(void);                        /* 显示打印机状态 */

/* V3.5 Phase 8: 调试统计模块 */
void crc_stats(void);        /* 显示CRC统计信息 */
void fifo_stats(void);       /* 显示FIFO统计信息 */

/* V3.7: 帮助系统 */
void motor_help(void);       /* 显示电机控制命令帮助 */

/* V3.6: TIM2实时定时器控制 */
#if REALTIME_MOTOR_ENABLE
void tim2_start(void);       /* 启动TIM2定时器 */
void tim2_stop(void);        /* 停止TIM2定时器 */
void tim2_status(void);      /* 显示TIM2状态 */
#endif

#endif /* __USMART_INTERFACE_H */
