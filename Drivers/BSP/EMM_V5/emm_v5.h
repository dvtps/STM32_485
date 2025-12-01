/**
 ****************************************************************************************************
 * @file        emm_v5.h
 * @author      张大头闭环伺服 + 正点原子团队适配
 * @version     V1.0 (HAL库版本)
 * @date        2025-12-01
 * @brief       Emm_V5.0步进闭环电机控制驱动（HAL库适配）
 * @license     基于张大头原始代码，适配正点原子M48Z-M3开发板
 ****************************************************************************************************
 * @attention
 * 
 * 原始作者: ZHANGDATOU
 * 技术支持: 张大头闭环伺服
 * 淘宝店铺: https://zhangdatou.taobao.com
 * CSDN博客: https://blog.csdn.net/zhangdatou666
 * qq交流群: 262438510
 * 
 * HAL库适配: 正点原子团队(ALIENTEK)
 * 实验平台: 正点原子 M48Z-M3最小系统板STM32F103版
 * 通信方式: RS485 (USART2, 115200bps)
 * 
 * 功能说明:
 * - 支持位置模式、速度模式控制
 * - 支持回零功能（单圈/多圈）
 * - 支持多机同步运动
 * - 支持参数读取和配置
 * 
 ****************************************************************************************************
 */

#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 宏定义 */
#define ABS(x)      ((x) > 0 ? (x) : -(x))          /* 绝对值宏 */

/* 系统参数类型枚举 */
typedef enum {
    S_VER   = 0,                                    /* 读取固件版本和对应的硬件版本 */
    S_RL    = 1,                                    /* 读取读取相电阻和相电感 */
    S_PID   = 2,                                    /* 读取PID参数 */
    S_VBUS  = 3,                                    /* 读取总线电压 */
    S_CPHA  = 5,                                    /* 读取相电流 */
    S_ENCL  = 7,                                    /* 读取经过线性化校准后的编码器值 */
    S_TPOS  = 8,                                    /* 读取电机目标位置角度 */
    S_VEL   = 9,                                    /* 读取电机实时转速 */
    S_CPOS  = 10,                                   /* 读取电机实时位置角度 */
    S_PERR  = 11,                                   /* 读取电机位置误差角度 */
    S_FLAG  = 13,                                   /* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 14,                                   /* 读取驱动参数 */
    S_State = 15,                                   /* 读取系统状态参数 */
    S_ORG   = 16,                                   /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;

/* 函数声明 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, 
                                  uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, 
                                  uint16_t sl_ma, uint16_t sl_ms, bool potF);
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_Origin_Interrupt(uint8_t addr);
uint32_t Emm_V5_Get_TX_Error_Count(void);                /* 获取UART发送错误计数 */
void emm_v5_tx_complete_callback(void);                  /* 发送完成回调(usart.c调用) */

#endif
