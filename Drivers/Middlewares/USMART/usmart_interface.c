/**
 ******************************************************************************
 * @file    usmart_interface.c
 * @author  STM32_485 Project
 * @version V3.1
 * @date    2025-12-01
 * @brief   USMART接口函数实现（桥接层）
 ******************************************************************************
 */

#include "usmart_interface.h"
#include "multi_motor_manager.h"
#include "protocol_router.h"
#include "emm_v5.h"
#include <stdio.h>

/* ============ 单电机控制实现 ============ */

void motor_enable(uint8_t addr, uint8_t enable)
{
    Emm_V5_En_Control(addr, enable ? true : false, false);
    printf("Motor#%d %s\r\n", addr, enable ? "ENABLED" : "DISABLED");
}

void motor_pos_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    Emm_V5_Pos_Control(addr, dir, speed, acc, pulses, false, false);
    printf("Motor#%d: POS dir=%d speed=%d pulses=%lu\r\n", addr, dir, speed, (unsigned long)pulses);
}

void motor_vel_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc)
{
    Emm_V5_Vel_Control(addr, dir, speed, acc, false);
    printf("Motor#%d: VEL dir=%d speed=%d\r\n", addr, dir, speed);
}

void motor_stop(uint8_t addr)
{
    Emm_V5_Stop_Now(addr, false);
    printf("Motor#%d: STOP\r\n", addr);
}

void motor_home(uint8_t addr)
{
    Emm_V5_Origin_Trigger_Return(addr, 0, false);
    printf("Motor#%d: HOME\r\n", addr);
}

void motor_read_status(uint8_t addr)
{
    Emm_V5_Read_Sys_Params(addr, false);
    printf("Motor#%d: STATUS query sent\r\n", addr);
}

/* ============ 多电机管理实现 ============ */

void multi_scan(uint8_t start, uint8_t end)
{
    int count = multi_motor_scan(start, end);
    printf("Scan complete: %d motors found\r\n", count);
}

void multi_map(uint8_t modbus, uint8_t physical)
{
    if (multi_motor_map_address(modbus, physical) == 0) {
        printf("Address mapped: Modbus#%d -> Physical#%d\r\n", modbus, physical);
    } else {
        printf("Map failed: invalid address\r\n");
    }
}

void multi_list(void)
{
    multi_motor_print_list();
}

void multi_enable(uint16_t mask, uint8_t enable)
{
    multi_motor_enable_batch(mask, enable ? true : false);
    printf("Batch enable: mask=0x%04X state=%d\r\n", mask, enable);
}

void multi_pos(uint16_t mask, uint8_t dir, uint16_t speed, uint32_t pulses)
{
    multi_motor_pos_control_batch(mask, dir, speed, 10, pulses);
    printf("Batch pos: mask=0x%04X dir=%d speed=%d pulses=%lu\r\n", 
           mask, dir, speed, (unsigned long)pulses);
}

void multi_vel(uint16_t mask, uint8_t dir, uint16_t speed)
{
    multi_motor_vel_control_batch(mask, dir, speed, 10);
    printf("Batch vel: mask=0x%04X dir=%d speed=%d\r\n", mask, dir, speed);
}

void multi_stop(uint16_t mask)
{
    multi_motor_stop_batch(mask);
    printf("Batch stop: mask=0x%04X\r\n", mask);
}

void multi_home(uint16_t mask, uint8_t mode)
{
    multi_motor_home_batch(mask, mode);
    printf("Batch home: mask=0x%04X mode=%d\r\n", mask, mode);
}

/* ============ 协议统计实现 ============ */

void proto_stats(void)
{
    const protocol_router_stats_t *stats = protocol_router_get_stats();
    
    printf("\r\n========== Protocol Statistics ==========\r\n");
    printf("Modbus RTU frames:  %lu\r\n", (unsigned long)stats->modbus_frames);
    printf("Emm_V5 frames:      %lu\r\n", (unsigned long)stats->emm_v5_frames);
    printf("Unknown frames:     %lu\r\n", (unsigned long)stats->unknown_frames);
    printf("CRC errors:         %lu\r\n", (unsigned long)stats->crc_errors);
    printf("=========================================\r\n\r\n");
}

void proto_reset(void)
{
    protocol_router_reset_stats();
    printf("Protocol statistics reset\r\n");
}
