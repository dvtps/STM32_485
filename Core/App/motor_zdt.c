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
#include "emm_v5.h"
#include "usart.h"

/* 私有变量 */
static uint32_t led_tick = 0;               /* LED闪烁时间戳 */
static uint32_t key_debounce_tick = 0;      /* 按键防抖时间戳 */
static uint32_t cmd_send_tick = 0;          /* 命令发送时间戳 */
static bool waiting_response = false;       /* 等待响应标志 */
static uint32_t motor_timeout_count = 0;    /* 电机超时计数 */

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
    uint8_t key;
    
    /* 检查电机响应超时 */
    if (waiting_response && 
        (HAL_GetTick() - cmd_send_tick) > MOTOR_RESPONSE_TIMEOUT_MS)
    {
        waiting_response = false;
        motor_timeout_count++;
        LOG_WARN("电机响应超时(第%u次)\r\n", (unsigned int)motor_timeout_count);
    }
    
    /* 按键扫描与处理 */
    key = key_scan(0);
    
    /* 按键防抖：至少间隔KEY_ACTION_DELAY_MS才响应 */
    if (key != 0 && (HAL_GetTick() - key_debounce_tick) > KEY_ACTION_DELAY_MS)
    {
        key_debounce_tick = HAL_GetTick();
        
        if (key == KEY0_PRES)                           /* KEY0按下：使能电机 */
        {
            Emm_V5_En_Control(MOTOR_DEFAULT_ADDRESS, true, false);
            LOG_INFO("电机使能\r\n");
        }
        else if (key == WKUP_PRES)                      /* WKUP按下：位置模式测试 */
        {
            /* 使用配置文件中的参数 */
            Emm_V5_Pos_Control(MOTOR_DEFAULT_ADDRESS, MOTOR_DEFAULT_DIR, 
                              MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_ACC, 
                              MOTOR_HALF_REV_PULSE, MOTOR_MODE_RELATIVE, false);
            LOG_INFO("电机运动: %d脉冲\r\n", MOTOR_HALF_REV_PULSE);
            
            /* 启动响应超时监控 */
            cmd_send_tick = HAL_GetTick();
            waiting_response = true;
        }
    }
    
    /* LED心跳指示系统运行（独立计时） */
    if ((HAL_GetTick() - led_tick) > LED_HEARTBEAT_PERIOD_MS)
    {
        led_tick = HAL_GetTick();
        LED0_TOGGLE();  /* 心跳闪烁 */
    }
    
    /* 处理电机响应帧 */
    if (g_emm_frame_complete)
    {
        g_emm_frame_complete = 0;
        waiting_response = false;  /* 收到响应，取消超时监控 */
        
        /* 完整帧验证：检查帧头、长度和校验字节 */
        if (g_emm_rx_count >= 4 && 
            g_emm_rx_cmd[0] == 0x01 && 
            g_emm_rx_cmd[g_emm_rx_count - 1] == 0x6B)
        {
            uint8_t addr = g_emm_rx_cmd[1];
            uint8_t cmd = g_emm_rx_cmd[2];
            
            /* CRC校验（Emm_V5.0协议：所有字节按位异或后应等于0x6B） */
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < g_emm_rx_count - 1; i++)
            {
                checksum ^= g_emm_rx_cmd[i];
            }
            
            if (checksum == 0x6B)
            {
#if DEBUG_MOTOR_RESPONSE
                LOG_DEBUG("电机响应: addr=0x%02X, cmd=0x%02X, 校验通过\r\n", addr, cmd);
#else
                (void)addr;
                (void)cmd;
#endif
            }
            else
            {
#if DEBUG_MOTOR_RESPONSE
                LOG_WARN("电机响应校验失败: 计算=0x%02X, 期望=0x6B\r\n", checksum);
#endif
            }
        }
        else if (g_emm_rx_count > 0)
        {
#if DEBUG_MOTOR_RESPONSE
            LOG_WARN("电机响应格式错误: len=%d, head=0x%02X, tail=0x%02X\r\n", 
                     g_emm_rx_count, g_emm_rx_cmd[0], 
                     g_emm_rx_cmd[g_emm_rx_count - 1]);
#endif
        }
    }
    
    /* 移除阻塞延时，提高主循环响应速度 */
}
