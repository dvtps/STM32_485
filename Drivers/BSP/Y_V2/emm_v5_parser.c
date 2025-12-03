/**
 ****************************************************************************************************
 * @file        emm_v5_parser.c
 * @author      STM32_485 Project Team
 * @version     V3.7
 * @date        2025-12-03
 * @brief       Emm_V5协议响应帧解析器实现（零拷贝高性能）
 ****************************************************************************************************
 */

#include "emm_v5_parser.h"
#include <string.h>

/* ======================== 私有变量 ======================== */

static emm_parser_stats_t g_parser_stats = {0};

/* ======================== 私有函数声明 ======================== */

static emm_response_type_t identify_frame_type(const uint8_t *rx_data, uint16_t len);

/* ======================== 公共函数实现 ======================== */

/**
 * @brief       初始化响应帧解析器
 */
void emm_parser_init(void)
{
    memset(&g_parser_stats, 0, sizeof(emm_parser_stats_t));
}

/**
 * @brief       解析接收到的响应帧（非阻塞）
 */
bool emm_parser_parse(const uint8_t *rx_data, uint16_t len, motor_response_t *result)
{
    if (!rx_data || !result || len < 3) {
        return false;  /* 参数错误或帧太短 */
    }
    
    g_parser_stats.total_frames++;
    
    /* 清空结果 */
    memset(result, 0, sizeof(motor_response_t));
    result->timestamp = HAL_GetTick();
    result->motor_addr = rx_data[0];
    
    /* 验证校验和（尾部0x6B）*/
    if (!emm_parser_verify_checksum(rx_data, len)) {
        g_parser_stats.checksum_errors++;
        result->type = EMM_RESP_ERROR;
        return false;
    }
    
    /* 识别帧类型 */
    result->type = identify_frame_type(rx_data, len);
    
    /* 根据类型解析数据 */
    switch (result->type) {
        case EMM_RESP_POSITION:
            result->valid = emm_parser_position(rx_data, len, &result->data.position);
            break;
            
        case EMM_RESP_VELOCITY:
            result->valid = emm_parser_velocity(rx_data, len, &result->data.velocity);
            break;
            
        case EMM_RESP_FLAG:
            result->valid = emm_parser_flag(rx_data, len, &result->data.flag);
            break;
            
        case EMM_RESP_PERR:
            result->valid = emm_parser_pos_error(rx_data, len, &result->data.pos_error);
            break;
            
        case EMM_RESP_ACK:
            /* 简单应答帧，无需解析数据 */
            result->valid = true;
            break;
            
        default:
            /* 未识别或错误帧 */
            result->valid = false;
            g_parser_stats.unknown_frames++;
            break;
    }
    
    if (result->valid) {
        g_parser_stats.valid_frames++;
    } else {
        g_parser_stats.invalid_frames++;
    }
    
    return result->valid;
}

/**
 * @brief       解析位置查询响应（S_CPOS, 功能码0x36）
 * @note        帧格式: [addr] [0x36] [pos_HH] [pos_HL] [pos_LH] [pos_LL] [0x6B]
 *              总长度: 7字节
 */
bool emm_parser_position(const uint8_t *rx_data, uint16_t len, int32_t *position)
{
    if (!rx_data || !position || len != 7) {
        return false;
    }
    
    /* 验证功能码 */
    if (rx_data[1] != 0x36) {
        return false;
    }
    
    /* 解析32位位置值（大端序）*/
    *position = ((int32_t)rx_data[2] << 24) |
                ((int32_t)rx_data[3] << 16) |
                ((int32_t)rx_data[4] << 8)  |
                ((int32_t)rx_data[5]);
    
    return true;
}

/**
 * @brief       解析速度查询响应（S_VEL, 功能码0x35）
 * @note        帧格式: [addr] [0x35] [vel_H] [vel_L] [0x6B]
 *              总长度: 5字节
 */
bool emm_parser_velocity(const uint8_t *rx_data, uint16_t len, int16_t *velocity)
{
    if (!rx_data || !velocity || len != 5) {
        return false;
    }
    
    /* 验证功能码 */
    if (rx_data[1] != 0x35) {
        return false;
    }
    
    /* 解析16位速度值（大端序）*/
    *velocity = ((int16_t)rx_data[2] << 8) | rx_data[3];
    
    return true;
}

/**
 * @brief       解析状态标志响应（S_FLAG, 功能码0x3A）
 * @note        帧格式: [addr] [0x3A] [flag_byte] [0x6B]
 *              总长度: 4字节
 */
bool emm_parser_flag(const uint8_t *rx_data, uint16_t len, motor_flag_t *flag)
{
    if (!rx_data || !flag || len != 4) {
        return false;
    }
    
    /* 验证功能码 */
    if (rx_data[1] != 0x3A) {
        return false;
    }
    
    /* 解析标志位（按位解析）*/
    uint8_t flag_byte = rx_data[2];
    flag->motor_enabled   = (flag_byte & 0x01) ? 1 : 0;
    flag->is_homing       = (flag_byte & 0x02) ? 1 : 0;
    flag->home_done       = (flag_byte & 0x04) ? 1 : 0;
    flag->is_clogged      = (flag_byte & 0x08) ? 1 : 0;
    flag->pos_error_over  = (flag_byte & 0x10) ? 1 : 0;
    flag->reserved        = (flag_byte >> 5) & 0x07;
    
    return true;
}

/**
 * @brief       解析位置误差响应（S_PERR, 功能码0x37）
 * @note        帧格式: [addr] [0x37] [err_HH] [err_HL] [err_LH] [err_LL] [0x6B]
 *              总长度: 7字节
 */
bool emm_parser_pos_error(const uint8_t *rx_data, uint16_t len, int32_t *error)
{
    if (!rx_data || !error || len != 7) {
        return false;
    }
    
    /* 验证功能码 */
    if (rx_data[1] != 0x37) {
        return false;
    }
    
    /* 解析32位误差值（大端序）*/
    *error = ((int32_t)rx_data[2] << 24) |
             ((int32_t)rx_data[3] << 16) |
             ((int32_t)rx_data[4] << 8)  |
             ((int32_t)rx_data[5]);
    
    return true;
}

/**
 * @brief       验证帧校验和（固定尾0x6B）
 */
bool emm_parser_verify_checksum(const uint8_t *rx_data, uint16_t len)
{
    if (!rx_data || len < 3) {
        return false;
    }
    
    /* Emm_V5协议使用固定校验字节0x6B */
    return (rx_data[len - 1] == 0x6B);
}

/**
 * @brief       获取解析器统计信息
 */
const emm_parser_stats_t* emm_parser_get_stats(void)
{
    return &g_parser_stats;
}

/**
 * @brief       重置解析器统计信息
 */
void emm_parser_reset_stats(void)
{
    memset(&g_parser_stats, 0, sizeof(emm_parser_stats_t));
}

/* ======================== 私有函数实现 ======================== */

/**
 * @brief       识别帧类型（根据功能码）
 * @param       rx_data: 帧数据
 * @param       len: 数据长度
 * @retval      帧类型枚举
 * @note        功能码定义：
 *              0x35 - S_VEL（速度）
 *              0x36 - S_CPOS（当前位置）
 *              0x37 - S_PERR（位置误差）
 *              0x3A - S_FLAG（状态标志）
 *              0x3B - S_ORG（回零状态）
 */
static emm_response_type_t identify_frame_type(const uint8_t *rx_data, uint16_t len)
{
    if (len < 3) {
        return EMM_RESP_ERROR;
    }
    
    uint8_t func_code = rx_data[1];
    
    switch (func_code) {
        case 0x35:
            return EMM_RESP_VELOCITY;   /* 速度查询响应 */
            
        case 0x36:
            return EMM_RESP_POSITION;   /* 位置查询响应 */
            
        case 0x37:
            return EMM_RESP_PERR;       /* 位置误差响应 */
            
        case 0x3A:
            return EMM_RESP_FLAG;       /* 状态标志响应 */
            
        case 0x3B:
            return EMM_RESP_ORG;        /* 回零状态响应 */
            
        /* 其他功能码可能是应答帧或未知帧 */
        default:
            if (len == 4) {
                return EMM_RESP_ACK;    /* 简单应答帧（长度4字节）*/
            }
            return EMM_RESP_UNKNOWN;
    }
}
