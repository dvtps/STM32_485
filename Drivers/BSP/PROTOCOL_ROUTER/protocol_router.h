/**
 ******************************************************************************
 * @file    protocol_router.h
 * @author  STM32_485 Project (BSP Layer)
 * @version V3.0
 * @date    2025-12-01
 * @brief   USART2协议路由器 - 自动识别Modbus RTU和Emm_V5协议
 ******************************************************************************
 * @attention
 * 
 * 架构设计：解决USART2多协议共存问题
 * - Modbus RTU: 站号1-247, 功能码0x03/0x04/0x06/0x10/0x17, CRC16校验
 * - Emm_V5:     地址0x00-0xFF, 命令码0xF3/0xFD/0xF6等, 校验字节
 * 
 * 识别策略：
 * 1. 帧长度判断（Modbus最短8字节，Emm_V5通常4-10字节）
 * 2. 功能码特征（Modbus固定范围，Emm_V5高字节为0xF*）
 * 3. CRC校验（Modbus使用CRC16-Modbus，Emm_V5使用简单校验和）
 * 
 ******************************************************************************
 */

#ifndef __PROTOCOL_ROUTER_H
#define __PROTOCOL_ROUTER_H

#include <stdint.h>
#include <stdbool.h>

/* 协议类型枚举 */
typedef enum {
    PROTOCOL_UNKNOWN = 0,   /* 未知协议 */
    PROTOCOL_MODBUS_RTU,    /* Modbus RTU协议 */
    PROTOCOL_EMM_V5,        /* 张大头Emm_V5协议 */
} protocol_type_t;

/* 协议路由统计（用于调试） */
typedef struct {
    uint32_t modbus_frames;     /* Modbus帧计数 */
    uint32_t emm_v5_frames;     /* Emm_V5帧计数 */
    uint32_t unknown_frames;    /* 未识别帧计数 */
    uint32_t crc_errors;        /* CRC校验失败计数 */
} protocol_router_stats_t;

/* ======================== 核心API ======================== */

/**
 * @brief       协议路由器初始化
 * @param       无
 * @retval      无
 */
void protocol_router_init(void);

/**
 * @brief       识别协议类型并路由到对应处理器
 * @param       data: 接收到的帧数据
 * @param       len: 帧长度
 * @retval      识别到的协议类型
 * @note        内部会自动分发到Modbus或Emm_V5处理器
 */
protocol_type_t protocol_router_process(const uint8_t *data, uint16_t len);

/**
 * @brief       获取路由统计信息
 * @param       无
 * @retval      统计信息结构体指针
 */
const protocol_router_stats_t* protocol_router_get_stats(void);

/**
 * @brief       重置路由统计
 * @param       无
 * @retval      无
 */
void protocol_router_reset_stats(void);

/* ======================== 协议识别辅助函数 ======================== */

/**
 * @brief       判断是否为有效的Modbus RTU帧
 * @param       data: 帧数据
 * @param       len: 帧长度
 * @retval      true: 是Modbus帧, false: 不是
 */
bool protocol_is_modbus_rtu(const uint8_t *data, uint16_t len);

/**
 * @brief       判断是否为有效的Emm_V5帧
 * @param       data: 帧数据
 * @param       len: 帧长度
 * @retval      true: 是Emm_V5帧, false: 不是
 */
bool protocol_is_emm_v5(const uint8_t *data, uint16_t len);

#endif /* __PROTOCOL_ROUTER_H */
