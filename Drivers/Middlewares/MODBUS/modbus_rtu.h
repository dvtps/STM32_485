/**
 ******************************************************************************
 * @file    modbus_rtu.h
 * @author  STM32_485 Project
 * @version V1.0
 * @date    2025-12-01
 * @brief   Modbus RTU协议解析器头文件
 *          提供CRC16校验、帧解析、功能码处理等核心功能
 ******************************************************************************
 * @attention
 * 本模块实现Modbus RTU协议的底层解析，支持以下功能码：
 * - 0x03: Read Holding Registers (读保持寄存器)
 * - 0x06: Write Single Register (写单个寄存器)
 * - 0x10: Write Multiple Registers (写多个寄存器)
 * - 0x04: Read Input Registers (读输入寄存器)
 * - 0x01: Read Coils (读线圈)
 * - 0x02: Read Discrete Inputs (读离散输入)
 * - 0x05: Write Single Coil (写单个线圈)
 * - 0x0F: Write Multiple Coils (写多个线圈)
 * - 0x17: Read/Write Multiple Registers (读写多个寄存器)
 ******************************************************************************
 */

#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================== 宏定义 ======================== */

/* Modbus RTU帧参数 */
#define MODBUS_RTU_MAX_FRAME_SIZE       256     /* 最大帧长度 */
#define MODBUS_RTU_MIN_FRAME_SIZE       4       /* 最小帧长度(地址+功能码+CRC16) */
#define MODBUS_RTU_CRC_SIZE             2       /* CRC校验字节数 */
#define MODBUS_RTU_TIMEOUT_MS           100     /* 帧超时时间(ms) */

/* Modbus功能码定义 */
#define MODBUS_FC_READ_COILS            0x01    /* 读线圈 */
#define MODBUS_FC_READ_DISCRETE         0x02    /* 读离散输入 */
#define MODBUS_FC_READ_HOLDING_REGS     0x03    /* 读保持寄存器 */
#define MODBUS_FC_READ_INPUT_REGS       0x04    /* 读输入寄存器 */
#define MODBUS_FC_WRITE_SINGLE_COIL     0x05    /* 写单个线圈 */
#define MODBUS_FC_WRITE_SINGLE_REG      0x06    /* 写单个寄存器 */
#define MODBUS_FC_WRITE_MULTIPLE_COILS  0x0F    /* 写多个线圈 */
#define MODBUS_FC_WRITE_MULTIPLE_REGS   0x10    /* 写多个寄存器 */
#define MODBUS_FC_READ_WRITE_REGS       0x17    /* 读写多个寄存器 */

/* Modbus异常码定义 */
#define MODBUS_EX_ILLEGAL_FUNCTION      0x01    /* 非法功能码 */
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS  0x02    /* 非法数据地址 */
#define MODBUS_EX_ILLEGAL_DATA_VALUE    0x03    /* 非法数据值 */
#define MODBUS_EX_SLAVE_DEVICE_FAILURE  0x04    /* 从机设备故障 */
#define MODBUS_EX_ACKNOWLEDGE           0x05    /* 确认 */
#define MODBUS_EX_SLAVE_DEVICE_BUSY     0x06    /* 从机设备忙 */
#define MODBUS_EX_MEMORY_PARITY_ERROR   0x08    /* 内存奇偶校验错误 */

/* 异常响应标志（功能码最高位置1） */
#define MODBUS_ERROR_BIT                0x80

/* ======================== 数据结构定义 ======================== */

/**
 * @brief Modbus RTU帧结构体
 */
typedef struct {
    uint8_t slave_addr;                         /* 从机地址(1-247) */
    uint8_t function_code;                      /* 功能码 */
    uint8_t data[MODBUS_RTU_MAX_FRAME_SIZE];    /* 数据域 */
    uint16_t data_len;                          /* 数据长度 */
    uint16_t crc;                               /* CRC校验值 */
} modbus_rtu_frame_t;

/**
 * @brief Modbus RTU解析状态枚举
 */
typedef enum {
    MODBUS_PARSE_OK = 0,            /* 解析成功 */
    MODBUS_PARSE_ERROR_CRC,         /* CRC校验错误 */
    MODBUS_PARSE_ERROR_LENGTH,      /* 帧长度错误 */
    MODBUS_PARSE_ERROR_ADDRESS,     /* 地址错误 */
    MODBUS_PARSE_ERROR_TIMEOUT,     /* 接收超时 */
} modbus_parse_status_t;

/**
 * @brief Modbus RTU接收缓冲区结构体
 */
typedef struct {
    uint8_t buffer[MODBUS_RTU_MAX_FRAME_SIZE];  /* 接收缓冲区 */
    uint16_t rx_index;                          /* 接收索引 */
    uint32_t last_rx_time;                      /* 最后接收时间(ms) */
    bool frame_complete;                        /* 帧完成标志 */
} modbus_rtu_rx_buffer_t;

/**
 * @brief Modbus RTU从机配置结构体
 */
typedef struct {
    uint8_t slave_address;          /* 从机地址(1-247, 0=广播) */
    uint32_t baudrate;              /* 波特率 */
    bool enabled;                   /* 使能标志 */
} modbus_rtu_config_t;

/* ======================== 函数声明 ======================== */

/**
 * @brief  Modbus RTU初始化
 * @param  slave_addr: 从机地址(1-247)
 * @param  baudrate: 波特率
 * @retval 0=成功, -1=失败
 */
int modbus_rtu_init(uint8_t slave_addr, uint32_t baudrate);

/**
 * @brief  计算Modbus CRC16校验值
 * @param  data: 数据缓冲区指针
 * @param  length: 数据长度
 * @retval CRC16校验值
 * @note   使用标准Modbus CRC16算法(多项式0xA001, 初值0xFFFF)
 */
uint16_t modbus_crc16(const uint8_t *data, uint16_t length);

/**
 * @brief  解析Modbus RTU帧
 * @param  rx_buffer: 接收缓冲区指针
 * @param  rx_len: 接收长度
 * @param  frame: 输出帧结构体指针
 * @retval modbus_parse_status_t 解析状态
 */
modbus_parse_status_t modbus_rtu_parse_frame(const uint8_t *rx_buffer, uint16_t rx_len, modbus_rtu_frame_t *frame);

/**
 * @brief  构建Modbus RTU响应帧
 * @param  frame: 帧结构体指针
 * @param  tx_buffer: 发送缓冲区指针
 * @param  tx_len: 输出发送长度指针
 * @retval 0=成功, -1=失败
 */
int modbus_rtu_build_response(const modbus_rtu_frame_t *frame, uint8_t *tx_buffer, uint16_t *tx_len);

/**
 * @brief  构建Modbus RTU异常响应帧
 * @param  slave_addr: 从机地址
 * @param  function_code: 功能码
 * @param  exception_code: 异常码
 * @param  tx_buffer: 发送缓冲区指针
 * @param  tx_len: 输出发送长度指针
 * @retval 0=成功, -1=失败
 */
int modbus_rtu_build_exception(uint8_t slave_addr, uint8_t function_code, uint8_t exception_code, 
                                uint8_t *tx_buffer, uint16_t *tx_len);
int modbus_rtu_build_exception(uint8_t slave_addr, uint8_t function_code, uint8_t exception_code,
                                uint8_t *tx_buffer, uint16_t *tx_len);

/**
 * @brief  处理Modbus RTU请求（核心处理函数）
 * @param  rx_buffer: 接收缓冲区指针
 * @param  rx_len: 接收长度
 * @param  tx_buffer: 发送缓冲区指针
 * @param  tx_len: 输出发送长度指针
 * @retval 0=成功, -1=失败（无需响应）, >0=异常码
 * @note   此函数会调用寄存器映射层的处理函数
 */
int modbus_rtu_process_request(const uint8_t *rx_buffer, uint16_t rx_len,
                                uint8_t *tx_buffer, uint16_t *tx_len);

/**
 * @brief  UART接收字节回调（由USART中断调用）
 * @param  byte: 接收到的字节
 * @retval None
 */
void modbus_rtu_rx_byte_callback(uint8_t byte);

/**
 * @brief  UART IDLE中断回调（由USART中断调用）
 * @param  None
 * @retval None
 * @note   检测到总线空闲，表示一帧接收完成
 */
void modbus_rtu_idle_callback(void);

/**
 * @brief  获取接收缓冲区指针（用于主循环处理）
 * @param  None
 * @retval 接收缓冲区结构体指针
 */
modbus_rtu_rx_buffer_t* modbus_rtu_get_rx_buffer(void);

/**
 * @brief  获取Modbus配置指针
 * @param  None
 * @retval 配置结构体指针
 */
modbus_rtu_config_t* modbus_rtu_get_config(void);

/**
 * @brief  设置从机地址
 * @param  slave_addr: 从机地址(1-247)
 * @retval 0=成功, -1=失败
 */
int modbus_rtu_set_slave_address(uint8_t slave_addr);

/**
 * @brief  获取从机地址
 * @param  None
 * @retval 从机地址
 */
uint8_t modbus_rtu_get_slave_address(void);

/* ======================== 调试宏定义 ======================== */

#if !defined(NDEBUG) && defined(DEBUG_ENABLE) && (DEBUG_ENABLE == 1)
    #define MODBUS_DEBUG_PRINTF(fmt, ...)   printf("[MODBUS] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_DEBUG_PRINTF(fmt, ...)   ((void)0)  /* Release: 移除所有printf调用 */
#endif

#endif /* __MODBUS_RTU_H */
