/**
 ****************************************************************************************************
 * @file        emm_v5.c
 * @author      张大头闭环伺服 + 正点原子团队适配
 * @version     V3.0 (使用统一的emm_uart通信层)
 * @date        2025-12-01
 * @brief       Emm_V5.0步进闭环电机控制驱动实现
 * @license     基于张大头原始代码，适配正点原子M48Z-M3开发板
 ****************************************************************************************************
 * @attention
 * 
 * 原始作者: ZHANGDATOU (https://zhangdatou.taobao.com)
 * HAL库适配: 正点原子团队(ALIENTEK)
 * 
 * 版本历史:
 * V1.0 - usart_SendCmd() (标准库)
 * V2.0 - HAL_UART_Transmit() (直接调用HAL)
 * V3.0 - emm_uart_send() (统一通信层，便于维护和扩展)
 * 
 ****************************************************************************************************
 */

#include "emm_v5.h"
#include "emm_uart.h"  /* 统一的RS485通信层 */
#include "usart.h"     /* 包含全局变量 g_emm_frame_complete */

/**
 * @brief       发送命令到电机（内部静态函数）
 * @param       cmd: 命令缓冲区
 * @param       len: 命令长度
 * @retval      0: 成功, 1: 失败, 2: 发送太频繁
 */
static uint8_t emm_v5_send_cmd(uint8_t *cmd, uint8_t len)
{
    uint8_t result;
    
    /* 调用统一的UART发送接口 */
    result = emm_uart_send(cmd, len);
    
    /* 清空帧完成标志，准备接收响应 */
    if (result == 0)
    {
        g_emm_frame_complete = 0;
    }
    
    return result;
}

/**
 * @brief       将当前位置清零
 * @param       addr: 电机地址
 * @retval      无
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0x0A;                              /* 功能码 */
    cmd[2] = 0x6D;                              /* 辅助码 */
    cmd[3] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 4);
}

/**
 * @brief       解除堵转保护
 * @param       addr: 电机地址
 * @retval      无
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0x0E;                              /* 功能码 */
    cmd[2] = 0x52;                              /* 辅助码 */
    cmd[3] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 4);
}

/**
 * @brief       读取系统参数
 * @param       addr: 电机地址
 * @param       s: 系统参数类型
 * @retval      无
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[i] = addr; ++i;                         /* 地址 */

    switch(s)                                   /* 功能码 */
    {
        case S_VER  : cmd[i] = 0x1F; ++i; break;
        case S_RL   : cmd[i] = 0x20; ++i; break;
        case S_PID  : cmd[i] = 0x21; ++i; break;
        case S_VBUS : cmd[i] = 0x24; ++i; break;
        case S_CPHA : cmd[i] = 0x27; ++i; break;
        case S_ENCL : cmd[i] = 0x31; ++i; break;
        case S_TPOS : cmd[i] = 0x33; ++i; break;
        case S_VEL  : cmd[i] = 0x35; ++i; break;
        case S_CPOS : cmd[i] = 0x36; ++i; break;
        case S_PERR : cmd[i] = 0x37; ++i; break;
        case S_FLAG : cmd[i] = 0x3A; ++i; break;
        case S_ORG  : cmd[i] = 0x3B; ++i; break;
        case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
        case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
        default: break;
    }

    cmd[i] = 0x6B; ++i;                         /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, i);
}

/**
 * @brief       修改开环/闭环控制模式
 * @param       addr: 电机地址
 * @param       svF: 是否存储标志，false为不存储，true为存储
 * @param       ctrl_mode: 控制模式，0关闭脉冲输入，1开环，2闭环，3多圈限位
 * @retval      无
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0x46;                              /* 功能码 */
    cmd[2] = 0x69;                              /* 辅助码 */
    cmd[3] = svF;                               /* 存储标志 */
    cmd[4] = ctrl_mode;                         /* 控制模式 */
    cmd[5] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 6);
}

/**
 * @brief       使能信号控制
 * @param       addr: 电机地址
 * @param       state: 使能状态，true为使能，false为关闭
 * @param       snF: 多机同步标志，false为不启用，true为启用
 * @retval      无
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0xF3;                              /* 功能码 */
    cmd[2] = 0xAB;                              /* 辅助码 */
    cmd[3] = (uint8_t)state;                    /* 使能状态 */
    cmd[4] = snF;                               /* 多机同步标志 */
    cmd[5] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 6);
}

/**
 * @brief       速度模式控制
 * @param       addr: 电机地址
 * @param       dir: 方向，0为CW，其余为CCW
 * @param       vel: 速度，范围0-5000 RPM
 * @param       acc: 加速度，范围0-255（0为直接启动）
 * @param       snF: 多机同步标志
 * @retval      无
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0xF6;                              /* 功能码 */
    cmd[2] = dir;                               /* 方向 */
    cmd[3] = (uint8_t)(vel >> 8);               /* 速度高8位 */
    cmd[4] = (uint8_t)(vel >> 0);               /* 速度低8位 */
    cmd[5] = acc;                               /* 加速度 */
    cmd[6] = snF;                               /* 多机同步标志 */
    cmd[7] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 8);
}

/**
 * @brief       位置模式控制
 * @param       addr: 电机地址
 * @param       dir: 方向，0为CW，其余为CCW
 * @param       vel: 速度，0-5000 RPM
 * @param       acc: 加速度，0-255
 * @param       clk: 脉冲数，0-(2^32-1)
 * @param       raF: 相对/绝对标志，false为相对，true为绝对
 * @param       snF: 多机同步标志
 * @retval      无
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    /* 装载命令 */
    cmd[0]  = addr;                             /* 地址 */
    cmd[1]  = 0xFD;                             /* 功能码 */
    cmd[2]  = dir;                              /* 方向 */
    cmd[3]  = (uint8_t)(vel >> 8);              /* 速度高8位 */
    cmd[4]  = (uint8_t)(vel >> 0);              /* 速度低8位 */
    cmd[5]  = acc;                              /* 加速度 */
    cmd[6]  = (uint8_t)(clk >> 24);             /* 脉冲数bit24-31 */
    cmd[7]  = (uint8_t)(clk >> 16);             /* 脉冲数bit16-23 */
    cmd[8]  = (uint8_t)(clk >> 8);              /* 脉冲数bit8-15 */
    cmd[9]  = (uint8_t)(clk >> 0);              /* 脉冲数bit0-7 */
    cmd[10] = raF;                              /* 相对/绝对标志 */
    cmd[11] = snF;                              /* 多机同步标志 */
    cmd[12] = 0x6B;                             /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 13);
}

/**
 * @brief       立即停止
 * @param       addr: 电机地址
 * @param       snF: 多机同步标志
 * @retval      无
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0xFE;                              /* 功能码 */
    cmd[2] = 0x98;                              /* 辅助码 */
    cmd[3] = snF;                               /* 多机同步标志 */
    cmd[4] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 5);
}

/**
 * @brief       多机同步运动
 * @param       addr: 电机地址
 * @retval      无
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0xFF;                              /* 功能码 */
    cmd[2] = 0x66;                              /* 辅助码 */
    cmd[3] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 4);
}

/**
 * @brief       设置单圈回零的零点位置
 * @param       addr: 电机地址
 * @param       svF: 是否存储，false不存储，true存储
 * @retval      无
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0x93;                              /* 功能码 */
    cmd[2] = 0x88;                              /* 辅助码 */
    cmd[3] = svF;                               /* 存储标志 */
    cmd[4] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 5);
}

/**
 * @brief       修改回零参数
 * @param       addr: 电机地址
 * @param       svF: 是否存储
 * @param       o_mode: 回零模式（0单圈就近，1单圈方向，2多圈无限位，3多圈有限位）
 * @param       o_dir: 回零方向，0为CW，其余为CCW
 * @param       o_vel: 回零速度 RPM
 * @param       o_tm: 回零超时时间 ms
 * @param       sl_vel: 无限位碰撞检测转速 RPM
 * @param       sl_ma: 无限位碰撞检测电流 mA
 * @param       sl_ms: 无限位碰撞检测时间 ms
 * @param       potF: 上电自动触发回零
 * @retval      无
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, 
                                  uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, 
                                  uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};
    
    /* 装载命令 */
    cmd[0]  = addr;                             /* 地址 */
    cmd[1]  = 0x4C;                             /* 功能码 */
    cmd[2]  = 0xAE;                             /* 辅助码 */
    cmd[3]  = svF;                              /* 存储标志 */
    cmd[4]  = o_mode;                           /* 回零模式 */
    cmd[5]  = o_dir;                            /* 回零方向 */
    cmd[6]  = (uint8_t)(o_vel >> 8);            /* 回零速度高8位 */
    cmd[7]  = (uint8_t)(o_vel >> 0);            /* 回零速度低8位 */
    cmd[8]  = (uint8_t)(o_tm >> 24);            /* 超时时间bit24-31 */
    cmd[9]  = (uint8_t)(o_tm >> 16);            /* 超时时间bit16-23 */
    cmd[10] = (uint8_t)(o_tm >> 8);             /* 超时时间bit8-15 */
    cmd[11] = (uint8_t)(o_tm >> 0);             /* 超时时间bit0-7 */
    cmd[12] = (uint8_t)(sl_vel >> 8);           /* 检测转速高8位 */
    cmd[13] = (uint8_t)(sl_vel >> 0);           /* 检测转速低8位 */
    cmd[14] = (uint8_t)(sl_ma >> 8);            /* 检测电流高8位 */
    cmd[15] = (uint8_t)(sl_ma >> 0);            /* 检测电流低8位 */
    cmd[16] = (uint8_t)(sl_ms >> 8);            /* 检测时间高8位 */
    cmd[17] = (uint8_t)(sl_ms >> 0);            /* 检测时间低8位 */
    cmd[18] = potF;                             /* 上电自动回零 */
    cmd[19] = 0x6B;                             /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 20);
}

/**
 * @brief       触发回零
 * @param       addr: 电机地址
 * @param       o_mode: 回零模式
 * @param       snF: 多机同步标志
 * @retval      无
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0x9A;                              /* 功能码 */
    cmd[2] = o_mode;                            /* 回零模式 */
    cmd[3] = snF;                               /* 多机同步标志 */
    cmd[4] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 5);
}

/**
 * @brief       强制中断并退出回零
 * @param       addr: 电机地址
 * @retval      无
 */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[0] = addr;                              /* 地址 */
    cmd[1] = 0x9C;                              /* 功能码 */
    cmd[2] = 0x48;                              /* 辅助码 */
    cmd[3] = 0x6B;                              /* 校验字节 */
    
    /* 发送命令 */
    emm_v5_send_cmd(cmd, 4);
}
