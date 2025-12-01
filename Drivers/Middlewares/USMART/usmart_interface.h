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
void motor_read_status(uint8_t addr);

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

#endif /* __USMART_INTERFACE_H */
