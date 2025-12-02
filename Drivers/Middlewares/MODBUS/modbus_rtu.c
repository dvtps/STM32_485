/**
 ******************************************************************************
 * @file    modbus_rtu.c
 * @author  STM32_485 Project
 * @version V1.0
 * @date    2025-12-01
 * @brief   Modbus RTU协议栈实现
 *          实现Modbus RTU核心协议解析、响应构造、CRC校验
 ******************************************************************************
 */

#include "app_config.h"

#if FEATURE_MODBUS_ENABLE

#include "modbus_rtu.h"
#include "modbus_gateway.h"
#include <string.h>

/* ======================== 私有变量 ======================== */

/* Modbus RTU配置 */
static modbus_rtu_config_t g_modbus_config = {
    .slave_address = 1,         /* 默认地址1 */
    .baudrate = 115200,         /* 默认波特率115200 */
    .enabled = false,           /* 初始化后使能 */
};

/* Modbus RTU接收缓冲区 */
static modbus_rtu_rx_buffer_t g_modbus_rx_buffer = {
    .rx_index = 0,
    .last_rx_time = 0,
    .frame_complete = false,
};

/* 统计信息 */
static struct {
    uint32_t rx_frame_count;    /* 接收帧计数 */
    uint32_t tx_frame_count;    /* 发送帧计数 */
    uint32_t crc_error_count;   /* CRC错误计数 */
    uint32_t timeout_count;     /* 超时计数 */
    uint32_t exception_count;   /* 异常响应计数 */
} g_modbus_stats = {0};

/* ======================== CRC16查找表（提高性能） ======================== */

/* Modbus标准CRC16查找表（多项式0xA001） */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* ======================== 私有函数声明 ======================== */

static int modbus_rtu_handle_read_holding_regs(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);
static int modbus_rtu_handle_write_single_reg(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);
static int modbus_rtu_handle_write_multiple_regs(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);
static int modbus_rtu_handle_read_input_regs(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);
static int modbus_rtu_handle_read_coils(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);
static int modbus_rtu_handle_read_discrete(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);
static int modbus_rtu_handle_write_single_coil(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);
static int modbus_rtu_handle_write_multiple_coils(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp);

/* ======================== 公共函数实现 ======================== */

/**
 * @brief  Modbus RTU初始化
 */
int modbus_rtu_init(uint8_t slave_addr, uint32_t baudrate)
{
    /* 参数检查 */
    if (slave_addr < 1 || slave_addr > 247) {
        return -1;
    }
    
    /* 配置参数 */
    g_modbus_config.slave_address = slave_addr;
    g_modbus_config.baudrate = baudrate;
    g_modbus_config.enabled = true;
    
    /* 清空接收缓冲区 */
    memset(&g_modbus_rx_buffer, 0, sizeof(g_modbus_rx_buffer));
    
    /* 清空统计信息 */
    memset(&g_modbus_stats, 0, sizeof(g_modbus_stats));
    
    MODBUS_DEBUG_PRINTF("Modbus RTU初始化完成: 地址=%d, 波特率=%u\r\n", 
                        slave_addr, (unsigned int)baudrate);
    
    return 0;
}

/**
 * @brief  计算Modbus CRC16校验值（查表法，高性能）
 */
uint16_t modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;  /* CRC初值 */
    uint16_t i;
    
    for (i = 0; i < length; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ data[i]) & 0xFF];
    }
    
    return crc;
}

/**
 * @brief  解析Modbus RTU帧
 */
modbus_parse_status_t modbus_rtu_parse_frame(const uint8_t *rx_buffer, uint16_t rx_len, modbus_rtu_frame_t *frame)
{
    uint16_t crc_calc, crc_recv;
    
    /* 检查最小长度 */
    if (rx_len < MODBUS_RTU_MIN_FRAME_SIZE) {
        return MODBUS_PARSE_ERROR_LENGTH;
    }
    
    /* 检查最大长度 */
    if (rx_len > MODBUS_RTU_MAX_FRAME_SIZE) {
        return MODBUS_PARSE_ERROR_LENGTH;
    }
    
    /* 提取从机地址 */
    frame->slave_addr = rx_buffer[0];
    
    /* 检查地址（0=广播地址也接受） */
    if (frame->slave_addr != 0 && frame->slave_addr != g_modbus_config.slave_address) {
        return MODBUS_PARSE_ERROR_ADDRESS;
    }
    
    /* 提取功能码 */
    frame->function_code = rx_buffer[1];
    
    /* 提取数据域 */
    frame->data_len = rx_len - MODBUS_RTU_MIN_FRAME_SIZE;
    if (frame->data_len > 0) {
        memcpy(frame->data, &rx_buffer[2], frame->data_len);
    }
    
    /* 提取CRC（低字节在前） */
    crc_recv = rx_buffer[rx_len - 2] | (rx_buffer[rx_len - 1] << 8);
    
    /* 计算CRC */
    crc_calc = modbus_crc16(rx_buffer, rx_len - 2);
    
    /* 校验CRC */
    if (crc_calc != crc_recv) {
        g_modbus_stats.crc_error_count++;
        MODBUS_DEBUG_PRINTF("CRC错误: 计算=0x%04X, 接收=0x%04X\r\n", crc_calc, crc_recv);
        return MODBUS_PARSE_ERROR_CRC;
    }
    
    frame->crc = crc_recv;
    
    return MODBUS_PARSE_OK;
}

/**
 * @brief  构建Modbus RTU响应帧
 */
int modbus_rtu_build_response(const modbus_rtu_frame_t *frame, uint8_t *tx_buffer, uint16_t *tx_len)
{
    uint16_t crc;
    uint16_t idx = 0;
    
    /* 从机地址 */
    tx_buffer[idx++] = frame->slave_addr;
    
    /* 功能码 */
    tx_buffer[idx++] = frame->function_code;
    
    /* 数据域 */
    if (frame->data_len > 0) {
        memcpy(&tx_buffer[idx], frame->data, frame->data_len);
        idx += frame->data_len;
    }
    
    /* 计算CRC */
    crc = modbus_crc16(tx_buffer, idx);
    
    /* 添加CRC（低字节在前） */
    tx_buffer[idx++] = crc & 0xFF;
    tx_buffer[idx++] = (crc >> 8) & 0xFF;
    
    *tx_len = idx;
    
    return 0;
}

/**
 * @brief  构建Modbus RTU异常响应帧
 */
int modbus_rtu_build_exception(uint8_t slave_addr, uint8_t function_code, uint8_t exception_code,
                                uint8_t *tx_buffer, uint16_t *tx_len)
{
    uint16_t crc;
    uint16_t idx = 0;
    
    /* 从机地址 */
    tx_buffer[idx++] = slave_addr;
    
    /* 功能码（最高位置1表示异常） */
    tx_buffer[idx++] = function_code | MODBUS_ERROR_BIT;
    
    /* 异常码 */
    tx_buffer[idx++] = exception_code;
    
    /* 计算CRC */
    crc = modbus_crc16(tx_buffer, idx);
    
    /* 添加CRC */
    tx_buffer[idx++] = crc & 0xFF;
    tx_buffer[idx++] = (crc >> 8) & 0xFF;
    
    *tx_len = idx;
    
    g_modbus_stats.exception_count++;
    
    MODBUS_DEBUG_PRINTF("异常响应: 功能码=0x%02X, 异常码=0x%02X\r\n", function_code, exception_code);
    
    return 0;
}

/**
 * @brief  处理Modbus RTU请求（核心调度函数）
 */
int modbus_rtu_process_request(const uint8_t *rx_buffer, uint16_t rx_len,
                                uint8_t *tx_buffer, uint16_t *tx_len)
{
    modbus_rtu_frame_t req_frame, resp_frame;
    modbus_parse_status_t parse_status;
    int ret;
    
    /* 解析请求帧 */
    parse_status = modbus_rtu_parse_frame(rx_buffer, rx_len, &req_frame);
    
    if (parse_status != MODBUS_PARSE_OK) {
        /* 解析错误，不响应 */
        return -1;
    }
    
    /* 广播地址不响应 */
    if (req_frame.slave_addr == 0) {
        MODBUS_DEBUG_PRINTF("%s\r\n", "广播请求，不响应");
        return -1;
    }
    
    g_modbus_stats.rx_frame_count++;
    
    /* 初始化响应帧 */
    memset(&resp_frame, 0, sizeof(resp_frame));
    resp_frame.slave_addr = req_frame.slave_addr;
    resp_frame.function_code = req_frame.function_code;
    
    /* 根据功能码调度处理 */
    switch (req_frame.function_code) {
        case MODBUS_FC_READ_HOLDING_REGS:
            ret = modbus_rtu_handle_read_holding_regs(&req_frame, &resp_frame);
            break;
            
        case MODBUS_FC_WRITE_SINGLE_REG:
            ret = modbus_rtu_handle_write_single_reg(&req_frame, &resp_frame);
            break;
            
        case MODBUS_FC_WRITE_MULTIPLE_REGS:
            ret = modbus_rtu_handle_write_multiple_regs(&req_frame, &resp_frame);
            break;
            
        case MODBUS_FC_READ_INPUT_REGS:
            ret = modbus_rtu_handle_read_input_regs(&req_frame, &resp_frame);
            break;
            
        case MODBUS_FC_READ_COILS:
            ret = modbus_rtu_handle_read_coils(&req_frame, &resp_frame);
            break;
            
        case MODBUS_FC_READ_DISCRETE:
            ret = modbus_rtu_handle_read_discrete(&req_frame, &resp_frame);
            break;
            
        case MODBUS_FC_WRITE_SINGLE_COIL:
            ret = modbus_rtu_handle_write_single_coil(&req_frame, &resp_frame);
            break;
            
        case MODBUS_FC_WRITE_MULTIPLE_COILS:
            ret = modbus_rtu_handle_write_multiple_coils(&req_frame, &resp_frame);
            break;
            
        default:
            /* 不支持的功能码 */
            ret = MODBUS_EX_ILLEGAL_FUNCTION;
            break;
    }
    
    /* 构建响应 */
    if (ret == 0) {
        /* 正常响应 */
        modbus_rtu_build_response(&resp_frame, tx_buffer, tx_len);
        g_modbus_stats.tx_frame_count++;
    } else {
        /* 异常响应 */
        modbus_rtu_build_exception(req_frame.slave_addr, req_frame.function_code, ret, tx_buffer, tx_len);
        g_modbus_stats.tx_frame_count++;
    }
    
    return ret;
}

/**
 * @brief  UART接收字节回调
 */
void modbus_rtu_rx_byte_callback(uint8_t byte)
{
    if (!g_modbus_config.enabled) {
        return;
    }
    
    /* 检查缓冲区溢出 */
    if (g_modbus_rx_buffer.rx_index >= MODBUS_RTU_MAX_FRAME_SIZE) {
        /* 溢出，重置缓冲区 */
        g_modbus_rx_buffer.rx_index = 0;
        return;
    }
    
    /* 存储字节 */
    g_modbus_rx_buffer.buffer[g_modbus_rx_buffer.rx_index++] = byte;
    
    /* 更新最后接收时间 */
    g_modbus_rx_buffer.last_rx_time = HAL_GetTick();
}

/**
 * @brief  UART IDLE中断回调
 */
void modbus_rtu_idle_callback(void)
{
    if (!g_modbus_config.enabled) {
        return;
    }
    
    /* 检查是否有数据 */
    if (g_modbus_rx_buffer.rx_index > 0) {
        /* 标记帧完成 */
        g_modbus_rx_buffer.frame_complete = true;
    }
}

/**
 * @brief  获取接收缓冲区指针
 */
modbus_rtu_rx_buffer_t* modbus_rtu_get_rx_buffer(void)
{
    return &g_modbus_rx_buffer;
}

/**
 * @brief  获取Modbus配置指针
 */
modbus_rtu_config_t* modbus_rtu_get_config(void)
{
    return &g_modbus_config;
}

/**
 * @brief  设置从机地址
 */
int modbus_rtu_set_slave_address(uint8_t slave_addr)
{
    if (slave_addr < 1 || slave_addr > 247) {
        return -1;
    }
    
    g_modbus_config.slave_address = slave_addr;
    return 0;
}

/**
 * @brief  获取从机地址
 */
uint8_t modbus_rtu_get_slave_address(void)
{
    return g_modbus_config.slave_address;
}

/* ======================== 私有函数实现（功能码处理） ======================== */

/**
 * @brief  处理0x03功能码：读保持寄存器
 */
static int modbus_rtu_handle_read_holding_regs(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t start_addr, num_regs;
    int ret;
    
    /* 检查数据长度 */
    if (req->data_len != 4) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取起始地址和寄存器数量 */
    start_addr = (req->data[0] << 8) | req->data[1];
    num_regs = (req->data[2] << 8) | req->data[3];
    
    /* 检查寄存器数量（1-125） */
    if (num_regs < 1 || num_regs > 125) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 调用寄存器映射层读取 */
    ret = modbus_gateway_read_holding_registers(start_addr, num_regs, &resp->data[1]);
    
    if (ret != 0) {
        return ret;  /* 返回异常码 */
    }
    
    /* 构建响应数据 */
    resp->data[0] = num_regs * 2;  /* 字节数 */
    resp->data_len = 1 + num_regs * 2;
    
    return 0;
}

/**
 * @brief  处理0x06功能码：写单个寄存器
 */
static int modbus_rtu_handle_write_single_reg(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t reg_addr, reg_value;
    int ret;
    
    /* 检查数据长度 */
    if (req->data_len != 4) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取寄存器地址和值 */
    reg_addr = (req->data[0] << 8) | req->data[1];
    reg_value = (req->data[2] << 8) | req->data[3];
    
    /* 调用寄存器映射层写入 */
    ret = modbus_gateway_write_single_register(reg_addr, reg_value);
    
    if (ret != 0) {
        return ret;  /* 返回异常码 */
    }
    
    /* 响应数据（回显请求） */
    memcpy(resp->data, req->data, 4);
    resp->data_len = 4;
    
    return 0;
}

/**
 * @brief  处理0x10功能码：写多个寄存器
 */
static int modbus_rtu_handle_write_multiple_regs(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t start_addr, num_regs, byte_count;
    int ret;
    
    /* 检查最小数据长度 */
    if (req->data_len < 5) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取参数 */
    start_addr = (req->data[0] << 8) | req->data[1];
    num_regs = (req->data[2] << 8) | req->data[3];
    byte_count = req->data[4];
    
    /* 检查寄存器数量（1-123） */
    if (num_regs < 1 || num_regs > 123) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 检查字节数 */
    if (byte_count != num_regs * 2) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 检查数据长度 */
    if (req->data_len != 5 + byte_count) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 调用寄存器映射层写入 */
    ret = modbus_gateway_write_multiple_registers(start_addr, num_regs, &req->data[5]);
    
    if (ret != 0) {
        return ret;  /* 返回异常码 */
    }
    
    /* 响应数据（回显起始地址和数量） */
    memcpy(resp->data, req->data, 4);
    resp->data_len = 4;
    
    return 0;
}

/**
 * @brief  处理0x04功能码：读输入寄存器
 */
static int modbus_rtu_handle_read_input_regs(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t start_addr, num_regs;
    int ret;
    
    /* 检查数据长度 */
    if (req->data_len != 4) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取参数 */
    start_addr = (req->data[0] << 8) | req->data[1];
    num_regs = (req->data[2] << 8) | req->data[3];
    
    /* 检查寄存器数量 */
    if (num_regs < 1 || num_regs > 125) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 调用寄存器映射层读取 */
    ret = modbus_gateway_read_input_registers(start_addr, num_regs, &resp->data[1]);
    
    if (ret != 0) {
        return ret;
    }
    
    /* 构建响应 */
    resp->data[0] = num_regs * 2;
    resp->data_len = 1 + num_regs * 2;
    
    return 0;
}

/**
 * @brief  处理0x01功能码：读线圈
 */
static int modbus_rtu_handle_read_coils(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t start_addr, num_coils;
    uint8_t byte_count;
    int ret;
    
    /* 检查数据长度 */
    if (req->data_len != 4) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取参数 */
    start_addr = (req->data[0] << 8) | req->data[1];
    num_coils = (req->data[2] << 8) | req->data[3];
    
    /* 检查线圈数量（1-2000） */
    if (num_coils < 1 || num_coils > 2000) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 计算字节数 */
    byte_count = (num_coils + 7) / 8;
    
    /* 调用寄存器映射层读取 */
    ret = modbus_gateway_read_coils(start_addr, num_coils, &resp->data[1]);
    
    if (ret != 0) {
        return ret;
    }
    
    /* 构建响应 */
    resp->data[0] = byte_count;
    resp->data_len = 1 + byte_count;
    
    return 0;
}

/**
 * @brief  处理0x02功能码：读离散输入
 */
static int modbus_rtu_handle_read_discrete(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t start_addr, num_inputs;
    uint8_t byte_count;
    int ret;
    
    /* 检查数据长度 */
    if (req->data_len != 4) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取参数 */
    start_addr = (req->data[0] << 8) | req->data[1];
    num_inputs = (req->data[2] << 8) | req->data[3];
    
    /* 检查数量 */
    if (num_inputs < 1 || num_inputs > 2000) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 计算字节数 */
    byte_count = (num_inputs + 7) / 8;
    
    /* 调用寄存器映射层读取 */
    ret = modbus_gateway_read_discrete_inputs(start_addr, num_inputs, &resp->data[1]);
    
    if (ret != 0) {
        return ret;
    }
    
    /* 构建响应 */
    resp->data[0] = byte_count;
    resp->data_len = 1 + byte_count;
    
    return 0;
}

/**
 * @brief  处理0x05功能码：写单个线圈
 */
static int modbus_rtu_handle_write_single_coil(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t coil_addr, coil_value;
    int ret;
    
    /* 检查数据长度 */
    if (req->data_len != 4) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取参数 */
    coil_addr = (req->data[0] << 8) | req->data[1];
    coil_value = (req->data[2] << 8) | req->data[3];
    
    /* 检查线圈值（0x0000或0xFF00） */
    if (coil_value != 0x0000 && coil_value != 0xFF00) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 调用寄存器映射层写入 */
    ret = modbus_gateway_write_single_coil(coil_addr, coil_value);
    
    if (ret != 0) {
        return ret;
    }
    
    /* 响应数据（回显请求） */
    memcpy(resp->data, req->data, 4);
    resp->data_len = 4;
    
    return 0;
}

/**
 * @brief  处理0x0F功能码：写多个线圈
 */
static int modbus_rtu_handle_write_multiple_coils(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    uint16_t start_addr, num_coils;
    uint8_t byte_count;
    int ret;
    
    /* 检查最小数据长度 */
    if (req->data_len < 5) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 提取参数 */
    start_addr = (req->data[0] << 8) | req->data[1];
    num_coils = (req->data[2] << 8) | req->data[3];
    byte_count = req->data[4];
    
    /* 检查线圈数量 */
    if (num_coils < 1 || num_coils > 1968) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 检查字节数 */
    if (byte_count != ((num_coils + 7) / 8)) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 检查数据长度 */
    if (req->data_len != 5 + byte_count) {
        return MODBUS_EX_ILLEGAL_DATA_VALUE;
    }
    
    /* 调用寄存器映射层写入 */
    ret = modbus_gateway_write_multiple_coils(start_addr, num_coils, &req->data[5]);
    
    if (ret != 0) {
        return ret;
    }
    
    /* 响应数据（回显起始地址和数量） */
    memcpy(resp->data, req->data, 4);
    resp->data_len = 4;
    
    return 0;
}

#endif /* FEATURE_MODBUS_ENABLE */
