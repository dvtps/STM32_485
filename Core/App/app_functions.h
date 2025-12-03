/**
 ****************************************************************************************************
 * @file        app_functions.h
 * @author      ZDT项目
 * @version     V1.0
 * @date        2025-12-03
 * @brief       USMART调试命令声明头文件（自动生成，Y_V2命令集集成）
 ****************************************************************************************************
 */
#ifndef __APP_FUNCTIONS_H
#define __APP_FUNCTIONS_H

#include "y_v2.h"


// Y_V2运动控制命令全量声明（与y_v2.h一致，便于USMART注册）
void Y_V2_Multi_Motor_Cmd(uint8_t addr);
void Y_V2_En_Control(uint8_t addr, bool state, bool snF);
void Y_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF);
void Y_V2_Torque_LV_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF, float maxVel);
void Y_V2_Vel_Control(uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF);
void Y_V2_Vel_LC_Control(uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF, uint16_t maxCur);
void Y_V2_Bypass_Pos_Control(uint8_t addr, uint8_t dir, float vel, float pos, uint8_t raf, bool snF);
void Y_V2_Bypass_Pos_LC_Control(uint8_t addr, uint8_t dir, float vel, float pos, uint8_t raf, bool snF, uint16_t maxCur);
void Y_V2_Traj_Pos_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float vel, float pos, uint8_t raf, bool snF);
void Y_V2_Traj_Pos_LC_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float vel, float pos, uint8_t raf, bool snF, uint16_t maxCur);
void Y_V2_Stop_Now(uint8_t addr, bool snF);
void Y_V2_Synchronous_Motion(uint8_t addr);

#endif
