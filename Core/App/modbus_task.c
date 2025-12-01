/**
 ******************************************************************************
 * @file    modbus_task.c
 * @author  STM32_485 Project (App Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   Modbus RTU任务处理模块（App层业务逻辑）
 ******************************************************************************
 * @attention
 * 
 * 架构定位：App层业务编排
 * - 从USART2接收缓冲区获取数据
 * - 调用Modbus RTU解析器处理请求
 * - 通过modbus_gateway映射到电机控制
 * - 发送响应帧
 * 
 ******************************************************************************
 */

#include "modbus_task.h"
#include "modbus_rtu.h"
#include "modbus_gateway.h"
#include "usart.h"
#include "app_config.h"
#include <string.h>

#if FEATURE_MODBUS_ENABLE

/* 外部变量：Modbus专用接收缓冲区（由protocol_router管理） */
extern uint8_t g_modbus_rx_buffer[256];
extern volatile uint16_t g_modbus_rx_count;
extern volatile uint8_t g_modbus_frame_complete;

/* 全局变量（V3.1: 已移除g_modbus_req_frame/g_modbus_resp_frame，由Middlewares层处理） */
static uint8_t g_modbus_tx_buffer[MODBUS_RTU_MAX_FRAME_SIZE]; /* 发送缓冲区 */

/**
 * @brief       Modbus任务初始化
 * @param       无
 * @retval      0: 成功, -1: 失败
 */
int modbus_task_init(void)
{
    /* 初始化Modbus RTU协议栈 */
    if (modbus_rtu_init(MODBUS_SLAVE_ADDRESS, MODBUS_BAUDRATE) != 0) {
        return -1;
    }
    
    /* 初始化Modbus Gateway */
    modbus_gateway_init();
    
    return 0;
}

/**
 * @brief       Modbus任务主循环（在main.c中调用）
 * @param       无
 * @retval      无
 * @note        V3.1架构优化：协议解析交由Middlewares层的modbus_rtu_process_request()处理
 *              App层仅负责缓冲区管理和UART发送调度
 */
void modbus_task_run(void)
{
    uint16_t tx_len = 0;
    
    /* V3.0: 检查Modbus专用缓冲区是否有完整帧 */
    if (!g_modbus_frame_complete) {
        return;
    }
    
    /* 清除完成标志 */
    g_modbus_frame_complete = 0;
    
    /* V3.1优化: 调用Middlewares层统一处理接口（解析+处理+构造响应） */
    int result = modbus_rtu_process_request(
        g_modbus_rx_buffer,     /* 输入：接收缓冲区 */
        g_modbus_rx_count,      /* 输入：接收长度 */
        g_modbus_tx_buffer,     /* 输出：发送缓冲区 */
        &tx_len                 /* 输出：发送长度 */
    );
    
    /* 如果需要发送响应（非广播且处理成功） */
    if (result == 0 && tx_len > 0) {
        /* 通过USART2发送响应 */
        HAL_UART_Transmit(&g_uart2_handle, g_modbus_tx_buffer, tx_len, 100);
    }
}

#endif /* FEATURE_MODBUS_ENABLE */
