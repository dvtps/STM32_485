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

/* 单电机控制（原motor_zdt.c封装） */
void motor_enable(uint8_t addr, uint8_t enable);
void motor_pos_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses);
void motor_vel_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc);
void motor_stop(uint8_t addr);
void motor_home(uint8_t addr);

/* 单电机查询（V3.5增强） */
void motor_read_status(uint8_t addr);    /* 读取状态标志 */
void motor_read_pos(uint8_t addr);       /* 读取当前位置 */
void motor_read_vel(uint8_t addr);       /* 读取当前速度 */
void motor_read_vbus(uint8_t addr);      /* 读取总线电压 */

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

#endif /* __USMART_INTERFACE_H */
