/**
 ****************************************************************************************************
 * @file        motor_zdt.c
 * @author      ZDT项目
 * @version     V1.0
 * @date        2025-12-01
 * @brief       ZDT电机控制应用层模块实现
 ****************************************************************************************************
 * @attention
 *
 * 功能说明:
 * - KEY0: 使能/失能1号电机
 * - WKUP: 位置模式测试(1圈=1600脉冲)
 * - LED: 500ms心跳闪烁
 *
 ****************************************************************************************************
 */

#include "motor_zdt.h"
#include "app_config.h"
#include "key.h"
#include "led.h"
#include "y_v2.h"      /* V3.0: Y系列X固件协议驱动 */
#include "usart.h"
#include "logger.h"

/* 私有变量 */
static uint32_t led_tick = 0;               /* LED闪烁时间戳 */
static bool dmx512_params_queried = false;  /* DMX512参数查询标志 */

/**
 * @brief       ZDT电机初始化
 * @param       无
 * @retval      无
 */
void motor_zdt_init(void)
{
    /* 预留:未来可添加电机参数配置、状态初始化等 */
    led_tick = HAL_GetTick();
}

/**
 * @brief       ZDT电机主循环
 * @param       无
 * @retval      无
 * @note        应在main函数的while(1)中循环调用
 */
void motor_zdt_run(void)
{
    /* 按配置参数自动下发命令（无按键控制） */
    static uint8_t step = 0;
    static uint32_t last_action_tick = 0;
    if ((HAL_GetTick() - last_action_tick) < 500) return;
    last_action_tick = HAL_GetTick();

    switch (step) {
        case 0: Y_V2_Trig_Encoder_Cal(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Trig_Encoder_Cal\r\n"); break;
        case 1: Y_V2_Reset_Motor(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Reset_Motor\r\n"); break;
        case 2: Y_V2_Reset_CurPos_To_Zero(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Reset_CurPos_To_Zero\r\n"); break;
        case 3: Y_V2_Reset_Clog_Pro(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Reset_Clog_Pro\r\n"); break;
        case 4: Y_V2_Restore_Motor(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Restore_Motor\r\n"); break;
        case 5: Y_V2_Multi_Motor_Cmd(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Multi_Motor_Cmd\r\n"); break;
        case 6: Y_V2_En_Control(MOTOR_DEFAULT_ADDRESS, true, false); LOG_INFO("Y_V2_En_Control\r\n"); break;
        case 7: Y_V2_Torque_Control(MOTOR_DEFAULT_ADDRESS, 0, 100, 2000, false); LOG_INFO("Y_V2_Torque_Control\r\n"); break;
        case 8: Y_V2_Torque_LV_Control(MOTOR_DEFAULT_ADDRESS, 0, 100, 2000, false, 300.0f); LOG_INFO("Y_V2_Torque_LV_Control\r\n"); break;
        case 9: Y_V2_Vel_Control(MOTOR_DEFAULT_ADDRESS, MOTOR_DEFAULT_DIR, MOTOR_DEFAULT_ACC, (float)MOTOR_DEFAULT_SPEED, false); LOG_INFO("Y_V2_Vel_Control\r\n"); break;
        case 10: Y_V2_Vel_LC_Control(MOTOR_DEFAULT_ADDRESS, MOTOR_DEFAULT_DIR, MOTOR_DEFAULT_ACC, (float)MOTOR_DEFAULT_SPEED, false, 2000); LOG_INFO("Y_V2_Vel_LC_Control\r\n"); break;
        case 11: Y_V2_Bypass_Pos_Control(MOTOR_DEFAULT_ADDRESS, MOTOR_DEFAULT_DIR, (float)MOTOR_DEFAULT_SPEED, 180.0f, MOTOR_MODE_RELATIVE, false); LOG_INFO("Y_V2_Bypass_Pos_Control\r\n"); break;
        case 12: Y_V2_Bypass_Pos_LC_Control(MOTOR_DEFAULT_ADDRESS, MOTOR_DEFAULT_DIR, (float)MOTOR_DEFAULT_SPEED, 180.0f, MOTOR_MODE_RELATIVE, false, 2000); LOG_INFO("Y_V2_Bypass_Pos_LC_Control\r\n"); break;
        case 13: Y_V2_Traj_Pos_Control(MOTOR_DEFAULT_ADDRESS, MOTOR_DEFAULT_DIR, MOTOR_DEFAULT_ACC, MOTOR_DEFAULT_ACC, (float)MOTOR_DEFAULT_SPEED, 180.0f, MOTOR_MODE_RELATIVE, false); LOG_INFO("Y_V2_Traj_Pos_Control\r\n"); break;
        case 14: Y_V2_Traj_Pos_LC_Control(MOTOR_DEFAULT_ADDRESS, MOTOR_DEFAULT_DIR, MOTOR_DEFAULT_ACC, MOTOR_DEFAULT_ACC, (float)MOTOR_DEFAULT_SPEED, 180.0f, MOTOR_MODE_RELATIVE, false, 2000); LOG_INFO("Y_V2_Traj_Pos_LC_Control\r\n"); break;
        case 15: Y_V2_Stop_Now(MOTOR_DEFAULT_ADDRESS, false); LOG_INFO("Y_V2_Stop_Now\r\n"); break;
        case 16: Y_V2_Synchronous_Motion(0); LOG_INFO("Y_V2_Synchronous_Motion\r\n"); break;
        case 17: Y_V2_Origin_Set_Zero(MOTOR_DEFAULT_ADDRESS, false); LOG_INFO("Y_V2_Origin_Set_Zero\r\n"); break;
        case 18: Y_V2_Origin_Trigger_Return(MOTOR_DEFAULT_ADDRESS, MOTOR_HOME_MODE, false); LOG_INFO("Y_V2_Origin_Trigger_Return\r\n"); break;
        case 19: Y_V2_Origin_Interrupt(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Origin_Interrupt\r\n"); break;
        case 20: Y_V2_Read_Origin_Params(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Read_Origin_Params\r\n"); break;
        case 21: Y_V2_Origin_Modify_Params(MOTOR_DEFAULT_ADDRESS, false, MOTOR_HOME_MODE, MOTOR_DEFAULT_DIR, MOTOR_HOME_SPEED, 1000, 100, 2000, 500, false); LOG_INFO("Y_V2_Origin_Modify_Params\r\n"); break;
        case 22: Y_V2_Auto_Return_Sys_Params_Timed(MOTOR_DEFAULT_ADDRESS, S_VEL, 1000); LOG_INFO("Y_V2_Auto_Return_Sys_Params_Timed\r\n"); break;
        case 23: Y_V2_Read_Sys_Params(MOTOR_DEFAULT_ADDRESS, S_VEL); LOG_INFO("Y_V2_Read_Sys_Params\r\n"); break;
        case 24: Y_V2_Modify_Motor_ID(MOTOR_DEFAULT_ADDRESS, false, 0x01); LOG_INFO("Y_V2_Modify_Motor_ID\r\n"); break;
        case 25: Y_V2_Modify_MicroStep(MOTOR_DEFAULT_ADDRESS, false, MOTOR_SUBDIVISION); LOG_INFO("Y_V2_Modify_MicroStep\r\n"); break;
        case 26: Y_V2_Modify_PDFlag(MOTOR_DEFAULT_ADDRESS, false); LOG_INFO("Y_V2_Modify_PDFlag\r\n"); break;
        case 27: Y_V2_Read_Opt_Param_Sta(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Read_Opt_Param_Sta\r\n"); break;
        case 28: Y_V2_Modify_Motor_Type(MOTOR_DEFAULT_ADDRESS, false, 0); LOG_INFO("Y_V2_Modify_Motor_Type\r\n"); break;
        case 29: Y_V2_Modify_Firmware_Type(MOTOR_DEFAULT_ADDRESS, false, 0); LOG_INFO("Y_V2_Modify_Firmware_Type\r\n"); break;
        case 30: Y_V2_Modify_Ctrl_Mode(MOTOR_DEFAULT_ADDRESS, false, 0); LOG_INFO("Y_V2_Modify_Ctrl_Mode\r\n"); break;
        case 31: Y_V2_Modify_Motor_Dir(MOTOR_DEFAULT_ADDRESS, false, MOTOR_DEFAULT_DIR); LOG_INFO("Y_V2_Modify_Motor_Dir\r\n"); break;
        case 32: Y_V2_Modify_Lock_Btn(MOTOR_DEFAULT_ADDRESS, false, 0); LOG_INFO("Y_V2_Modify_Lock_Btn\r\n"); break;
        case 33: Y_V2_Modify_S_Vel(MOTOR_DEFAULT_ADDRESS, false, 0); LOG_INFO("Y_V2_Modify_S_Vel\r\n"); break;
        case 34: Y_V2_Modify_OM_mA(MOTOR_DEFAULT_ADDRESS, false, 1000); LOG_INFO("Y_V2_Modify_OM_mA\r\n"); break;
        case 35: Y_V2_Modify_FOC_mA(MOTOR_DEFAULT_ADDRESS, false, 2000); LOG_INFO("Y_V2_Modify_FOC_mA\r\n"); break;
        case 36: Y_V2_Read_PID_Params(MOTOR_DEFAULT_ADDRESS); LOG_INFO("Y_V2_Read_PID_Params\r\n"); break;
        case 37: Y_V2_Modify_PID_Params(MOTOR_DEFAULT_ADDRESS, false, 100, 10, 1); LOG_INFO("Y_V2_Modify_PID_Params\r\n"); break;
        case 38:
            if (!dmx512_params_queried) {
                Y_V2_Read_DMX512_Params(MOTOR_DEFAULT_ADDRESS);
                LOG_INFO("Y_V2_Read_DMX512_Params\r\n");
                dmx512_params_queried = true;
            }
            break;
        default: break;
    }
    if (step < 38) step++;
        (void)dmx512_params_queried; // 防止未使用变量警告
    
    /* LED心跳指示系统运行（独立计时） */
    if ((HAL_GetTick() - led_tick) > LED_HEARTBEAT_PERIOD_MS)
    {
        led_tick = HAL_GetTick();
        LED0_TOGGLE();  /* 心跳闪烁 */
    }
    
    /* V3.7: 响应帧处理已迁移到 motor_monitor 系统，此处不再处理 */
    /* 如果需要响应超时监控，应通过 motor_monitor_is_online() 查询电机状态 */
    
    /* 移除阻塞延时，提高主循环响应速度 */
    (void)dmx512_params_queried; // 防止未使用变量警告
}
