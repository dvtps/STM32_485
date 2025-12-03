/**
 ****************************************************************************************************
 * @file        emm_v5_parser.h
 * @author      STM32_485 Project Team
 * @version     V3.7
 * @date        2025-12-03
 * @brief       Emm_V5协议响应帧解析器（非阻塞异步设计）
 ****************************************************************************************************
 * @attention   设计原则：
 *              1. 非阻塞：所有解析函数立即返回，不等待
 *              2. 异步查询：主循环轮询，不影响实时性
 *              3. 零拷贝：直接解析FIFO数据，无额外缓冲
 ****************************************************************************************************
 */

#ifndef __EMM_V5_PARSER_H
#define __EMM_V5_PARSER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ======================== 响应帧类型定义 ======================== */

/**
 * @brief       响应帧类型枚举
 */
typedef enum {
    EMM_RESP_NONE = 0,      /* 无响应 */
    EMM_RESP_ACK,           /* 应答帧（命令接收确认）*/
    EMM_RESP_POSITION,      /* 位置查询响应（S_CPOS）*/
    EMM_RESP_VELOCITY,      /* 速度查询响应（S_VEL）*/
    EMM_RESP_FLAG,          /* 状态标志响应（S_FLAG）*/
    EMM_RESP_PERR,          /* 位置误差响应（S_PERR）*/
    EMM_RESP_ORG,           /* 回零状态响应（S_ORG）*/
    EMM_RESP_ERROR,         /* 错误帧 */
    EMM_RESP_UNKNOWN        /* 未知帧 */
} emm_response_type_t;

/**
 * @brief       电机状态标志位（S_FLAG响应解析结果）
 */
typedef struct {
    uint8_t motor_enabled   : 1;    /* 使能状态（1=使能）*/
    uint8_t is_homing       : 1;    /* 回零中（1=正在回零）*/
    uint8_t home_done       : 1;    /* 回零完成（1=已完成）*/
    uint8_t is_clogged      : 1;    /* 堵转（1=检测到堵转）*/
    uint8_t pos_error_over  : 1;    /* 位置误差过大（1=误差>阈值）*/
    uint8_t reserved        : 3;    /* 保留位 */
} motor_flag_t;

/**
 * @brief       响应帧解析结果（统一数据结构）
 */
typedef struct {
    emm_response_type_t type;       /* 响应类型 */
    uint8_t motor_addr;             /* 电机地址 */
    bool valid;                     /* 数据有效性 */
    uint32_t timestamp;             /* 接收时间戳 */
    
    union {
        int32_t position;           /* 位置值（S_CPOS, 脉冲数）*/
        int16_t velocity;           /* 速度值（S_VEL, RPM）*/
        int32_t pos_error;          /* 位置误差（S_PERR）*/
        motor_flag_t flag;          /* 状态标志（S_FLAG）*/
        uint8_t org_status;         /* 回零状态（S_ORG）*/
        uint8_t raw_data[16];       /* 原始数据（用于未解析帧）*/
    } data;
} motor_response_t;

/* ======================== 公共函数声明 ======================== */

/**
 * @brief       初始化响应帧解析器
 * @param       无
 * @retval      无
 */
void emm_parser_init(void);

/**
 * @brief       解析接收到的响应帧（非阻塞）
 * @param       rx_data: 接收缓冲区指针
 * @param       len: 数据长度
 * @param       result: 解析结果输出（调用者分配）
 * @retval      解析是否成功（true=成功，false=失败或未识别）
 * @note        ⚡ 此函数立即返回，不等待数据
 */
bool emm_parser_parse(const uint8_t *rx_data, uint16_t len, motor_response_t *result);

/**
 * @brief       解析位置查询响应（S_CPOS, 功能码0x36）
 * @param       rx_data: 响应帧数据
 * @param       len: 数据长度
 * @param       position: 解析出的位置值（输出）
 * @retval      true=解析成功，false=帧格式错误
 * @note        帧格式: [addr] [0x36] [pos_HH] [pos_HL] [pos_LH] [pos_LL] [0x6B]
 *              位置范围: -2147483648 ~ 2147483647（有符号32位）
 */
bool emm_parser_position(const uint8_t *rx_data, uint16_t len, int32_t *position);

/**
 * @brief       解析速度查询响应（S_VEL, 功能码0x35）
 * @param       rx_data: 响应帧数据
 * @param       len: 数据长度
 * @param       velocity: 解析出的速度值（输出，RPM）
 * @retval      true=解析成功，false=帧格式错误
 * @note        帧格式: [addr] [0x35] [vel_H] [vel_L] [0x6B]
 *              速度范围: -5000 ~ 5000 RPM（有符号16位）
 */
bool emm_parser_velocity(const uint8_t *rx_data, uint16_t len, int16_t *velocity);

/**
 * @brief       解析状态标志响应（S_FLAG, 功能码0x3A）
 * @param       rx_data: 响应帧数据
 * @param       len: 数据长度
 * @param       flag: 解析出的状态标志（输出）
 * @retval      true=解析成功，false=帧格式错误
 * @note        帧格式: [addr] [0x3A] [flag_byte] [0x6B]
 */
bool emm_parser_flag(const uint8_t *rx_data, uint16_t len, motor_flag_t *flag);

/**
 * @brief       解析位置误差响应（S_PERR, 功能码0x37）
 * @param       rx_data: 响应帧数据
 * @param       len: 数据长度
 * @param       error: 解析出的位置误差（输出）
 * @retval      true=解析成功，false=帧格式错误
 * @note        帧格式: [addr] [0x37] [err_HH] [err_HL] [err_LH] [err_LL] [0x6B]
 */
bool emm_parser_pos_error(const uint8_t *rx_data, uint16_t len, int32_t *error);

/**
 * @brief       验证帧校验和（固定尾0x6B）
 * @param       rx_data: 帧数据
 * @param       len: 数据长度
 * @retval      true=校验通过，false=校验失败
 */
bool emm_parser_verify_checksum(const uint8_t *rx_data, uint16_t len);

/**
 * @brief       获取解析器统计信息
 * @param       无
 * @retval      统计信息结构体指针
 */
typedef struct {
    uint32_t total_frames;      /* 总接收帧数 */
    uint32_t valid_frames;      /* 有效帧数 */
    uint32_t invalid_frames;    /* 无效帧数 */
    uint32_t checksum_errors;   /* 校验错误数 */
    uint32_t unknown_frames;    /* 未识别帧数 */
} emm_parser_stats_t;

const emm_parser_stats_t* emm_parser_get_stats(void);

/**
 * @brief       重置解析器统计信息
 * @param       无
 * @retval      无
 */
void emm_parser_reset_stats(void);

#endif /* __EMM_V5_PARSER_H */
