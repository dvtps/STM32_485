/**
 ******************************************************************************
 * @file    protocol_router.c
 * @author  STM32_485 Project (BSP Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   USART2协议路由器实现
 ******************************************************************************
 */

#include "protocol_router.h"
#include "app_config.h"
#include "usart.h"              /* 包含USART2_REC_LEN定义 */
#include <string.h>

/* 外部变量声明（来自各协议模块，使用usart.h定义） */
extern uint8_t g_emm_rx_cmd[USART2_REC_LEN];    /* Emm_V5接收缓冲区(220字节) */
extern uint16_t g_emm_rx_count;                  /* Emm_V5接收字节数 */
extern volatile uint8_t g_emm_frame_complete;    /* Emm_V5帧完成标志 */

/* Modbus RTU专用缓冲区（新增） */
uint8_t g_modbus_rx_buffer[256];            /* Modbus接收缓冲区 */
volatile uint16_t g_modbus_rx_count = 0;    /* Modbus接收字节数 */
volatile uint8_t g_modbus_frame_complete = 0; /* Modbus帧完成标志 */

/* 路由统计 */
static protocol_router_stats_t g_router_stats = {0};

/* ======================== Modbus CRC16计算 ======================== */

/**
 * @brief       计算Modbus CRC16校验码
 * @param       data: 数据指针
 * @param       len: 数据长度
 * @retval      CRC16校验码
 */
static uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/* ======================== 协议识别函数 ======================== */

/**
 * @brief       判断是否为有效的Modbus RTU帧
 * @param       data: 帧数据
 * @param       len: 帧长度
 * @retval      true: 是Modbus帧, false: 不是
 */
bool protocol_is_modbus_rtu(const uint8_t *data, uint16_t len)
{
    /* 基本长度检查：Modbus最短8字节（地址+功能码+数据2字节+CRC2字节） */
    if (len < 8) {
        return false;
    }
    
    /* 站号检查：1-247为有效从机地址（0为广播地址也支持） */
    uint8_t addr = data[0];
    if (addr == 0 || (addr >= 1 && addr <= 247)) {
        /* 功能码检查：Modbus标准功能码 */
        uint8_t func = data[1];
        if (func == 0x03 || func == 0x04 || func == 0x06 || 
            func == 0x10 || func == 0x17 || func == 0x16) {
            
            /* CRC校验：Modbus使用CRC16-Modbus（低字节在前） */
            uint16_t recv_crc = data[len - 2] | (data[len - 1] << 8);
            uint16_t calc_crc = modbus_crc16(data, len - 2);
            
            if (recv_crc == calc_crc) {
                return true;  /* CRC正确，确认为Modbus帧 */
            } else {
                g_router_stats.crc_errors++;
            }
        }
    }
    
    return false;
}

/**
 * @brief       判断是否为有效的Emm_V5帧
 * @param       data: 帧数据
 * @param       len: 帧长度
 * @retval      true: 是Emm_V5帧, false: 不是
 */
bool protocol_is_emm_v5(const uint8_t *data, uint16_t len)
{
    /* Emm_V5帧格式：
     * 响应帧：0x01 + 地址 + 命令码 + 校验字节 (4字节)
     * 查询帧：0x01 + 地址 + 命令码 + 数据 + 校验字节 (变长)
     */
    
    /* 基本长度检查：Emm_V5最短4字节 */
    if (len < 4 || len > 32) {
        return false;
    }
    
    /* 帧头检查：第一个字节通常是0x01 */
    if (data[0] != 0x01) {
        return false;
    }
    
    /* 地址范围检查：0x00-0xFF都有效（广播地址0x00） */
    uint8_t addr = data[1];
    (void)addr;  /* 地址范围已在0x00-0xFF内 */
    
    /* 命令码检查：Emm_V5命令码通常高字节为0xF* */
    uint8_t cmd = data[2];
    if (cmd == 0xF3 || cmd == 0xF6 || cmd == 0xFD || cmd == 0xFF || 
        cmd == 0xF0 || cmd == 0xE6 || cmd == 0xE7 || cmd == 0xE8) {
        
        /* 简单校验和验证（Emm_V5使用最后一字节作为校验） */
        uint8_t checksum = 0;
        for (uint16_t i = 0; i < len - 1; i++) {
            checksum += data[i];
        }
        
        if (checksum == data[len - 1]) {
            return true;  /* 校验通过，确认为Emm_V5帧 */
        }
    }
    
    return false;
}

/* ======================== 协议路由核心 ======================== */

/**
 * @brief       协议路由器初始化
 * @param       无
 * @retval      无
 */
void protocol_router_init(void)
{
    memset(&g_router_stats, 0, sizeof(g_router_stats));
    g_modbus_rx_count = 0;
    g_modbus_frame_complete = 0;
}

/**
 * @brief       识别协议类型并路由到对应处理器
 * @param       data: 接收到的帧数据
 * @param       len: 帧长度
 * @retval      识别到的协议类型
 */
protocol_type_t protocol_router_process(const uint8_t *data, uint16_t len)
{
    /* 优先级1: 检查Modbus RTU（严格CRC校验，误判率低） */
    if (protocol_is_modbus_rtu(data, len)) {
        /* 复制到Modbus专用缓冲区 */
        if (len <= sizeof(g_modbus_rx_buffer)) {
            memcpy(g_modbus_rx_buffer, data, len);
            g_modbus_rx_count = len;
            g_modbus_frame_complete = 1;  /* 通知modbus_task处理 */
        }
        
        g_router_stats.modbus_frames++;
        return PROTOCOL_MODBUS_RTU;
    }
    
    /* 优先级2: 检查Emm_V5 */
    if (protocol_is_emm_v5(data, len)) {
        /* 复制到Emm_V5缓冲区（保持兼容性） */
        if (len <= sizeof(g_emm_rx_cmd)) {
            memcpy(g_emm_rx_cmd, data, len);
            g_emm_rx_count = len;
            g_emm_frame_complete = 1;  /* 通知motor_zdt处理 */
        }
        
        /* V3.5 Phase 5: 处理电机查询响应（调用modbus_gateway解析） */
        extern void modbus_gateway_handle_motor_response(uint8_t motor_addr, const uint8_t *data, uint16_t len);
        if (len >= 4 && data[0] >= 1 && data[0] <= 8) {
            modbus_gateway_handle_motor_response(data[0], data, len);
        }
        
        g_router_stats.emm_v5_frames++;
        return PROTOCOL_EMM_V5;
    }
    
    /* 未识别协议 */
    g_router_stats.unknown_frames++;
    return PROTOCOL_UNKNOWN;
}

/**
 * @brief       获取路由统计信息
 * @param       无
 * @retval      统计信息结构体指针
 */
const protocol_router_stats_t* protocol_router_get_stats(void)
{
    return &g_router_stats;
}

/**
 * @brief       重置路由统计
 * @param       无
 * @retval      无
 */
void protocol_router_reset_stats(void)
{
    memset(&g_router_stats, 0, sizeof(g_router_stats));
}
