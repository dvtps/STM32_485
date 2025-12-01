/**
 ****************************************************************************************************
 * @file        emm_v5.c
 * @author      张大头闭环伺�?+ 正点原子团队适配
 * @version     V3.0 (使用统一的emm_uart通信�?
 * @date        2025-12-01
 * @brief       Emm_V5.0步进闭环电机控制驱动实现
 * @license     基于张大头原始代码，适配正点原子M48Z-M3开发板
 ****************************************************************************************************
 * @attention
 * 
 * 原始作�? ZHANGDATOU (https://zhangdatou.taobao.com)
 * HAL库适配: 正点原子团队(ALIENTEK)
 * 
 * 版本历史:
 * V1.0 - usart_SendCmd() (标准�?
 * V2.0 - HAL_UART_Transmit() (直接调用HAL)
 * V3.0 - emm_uart_send() (统一通信层，便于维护和扩�?
 * 
 ****************************************************************************************************
 */

#include "emm_v5.h"
#include "emm_uart.h"  /* 统一的RS485通信�?*/
#include "usart.h"     /* 包含全局变量 g_emm_frame_complete */
#include "error_handler.h"  /* V3.5 Phase 3: 参数验证�?*/

/**
 * @brief       发送命令宏（内联优化，减少函数调用开销�?
 * @note        替代原static函数，从15次函数调用→直接内联
 */
#define EMM_V5_SEND_CMD(cmd, len) \
    do { \
        uint8_t _result = emm_uart_send((cmd), (len)); \
        if (_result == 0) g_emm_frame_complete = 0; \
    } while(0)

/**
 * @brief       将当前位置清�?
 * @param       addr: 电机地址
 * @retval      �?
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);  /* 地址1-255，不支持广播地址0 */
    
    uint8_t cmd[4];
    cmd[0] = addr; cmd[1] = 0x0A; cmd[2] = 0x6D; cmd[3] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 4);
}

/**
 * @brief       解除堵转保护
 * @param       addr: 电机地址
 * @retval      �?
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    
    uint8_t cmd[4];
    cmd[0] = addr; cmd[1] = 0x0E; cmd[2] = 0x52; cmd[3] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 4);
}

/**
 * @brief       读取系统参数
 * @param       addr: 电机地址
 * @param       s: 系统参数类型
 * @retval      �?
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    CHECK_PARAM_VOID(s <= S_ORG);  /* 系统参数类型0-16 */
    
    uint8_t i = 0;
    uint8_t cmd[16] = {0};
    
    /* 装载命令 */
    cmd[i] = addr; ++i;                         /* 地址 */

    switch(s)                                   /* 功能�?*/
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

    cmd[i] = EMM_V5_CHECKSUM; ++i;              /* 校验字节 */
    
    /* 发送命�?*/
    EMM_V5_SEND_CMD(cmd, i);
}

/**
 * @brief       修改开�?闭环控制模式
 * @param       addr: 电机地址
 * @param       svF: 是否存储标志，false为不存储，true为存�?
 * @param       ctrl_mode: 控制模式�?关闭脉冲输入�?开环，2闭环�?多圈限位
 * @retval      �?
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    CHECK_PARAM_VOID(ctrl_mode <= 3);  /* 控制模式0-3：关�?开�?闭环/多圈限位 */
    
    uint8_t cmd[6];
    cmd[0] = addr; cmd[1] = 0x46; cmd[2] = 0x69;
    cmd[3] = svF; cmd[4] = ctrl_mode; cmd[5] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 6);
}

/**
 * @brief       使能信号控制
 * @param       addr: 电机地址
 * @param       state: 使能状态，true为使能，false为关�?
 * @param       snF: 多机同步标志，false为不启用，true为启�?
 * @retval      �?
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 0 && addr <= 255);  /* 支持广播地址0 */
    
    uint8_t cmd[6];
    cmd[0] = addr; cmd[1] = 0xF3; cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state; cmd[4] = snF; cmd[5] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 6);
}

/**
 * @brief       速度模式控制
 * @param       addr: 电机地址
 * @param       dir: 方向�?为CW，其余为CCW
 * @param       vel: 速度，范�?-5000 RPM
 * @param       acc: 加速度，范�?-255�?为直接启动）
 * @param       snF: 多机同步标志
 * @retval      �?
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    CHECK_PARAM_VOID(vel <= 5000);  /* 速度0-5000 RPM */
    /* acc范围0-255（uint8_t自动限制），dir无需验证�?=CW,其余=CCW�?*/
    
    uint8_t cmd[8];
    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel);
    cmd[5] = acc;
    cmd[6] = snF;
    cmd[7] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 8);
}

/**
 * @brief       位置模式控制
 * @param       addr: 电机地址
 * @param       dir: 方向�?为CW，其余为CCW
 * @param       vel: 速度�?-5000 RPM
 * @param       acc: 加速度�?-255
 * @param       clk: 脉冲数，0-(2^32-1)
 * @param       raF: 相对/绝对标志，false为相对，true为绝�?
 * @param       snF: 多机同步标志
 * @retval      �?
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    CHECK_PARAM_VOID(vel <= 5000);  /* 速度0-5000 RPM */
    /* clk范围0-(2^32-1)（uint32_t自动限制�?*/
    
    uint8_t cmd[13];
    cmd[0] = addr;
    cmd[1] = 0xFD;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel);
    cmd[5] = acc;
    cmd[6] = (uint8_t)(clk >> 24);
    cmd[7] = (uint8_t)(clk >> 16);
    cmd[8] = (uint8_t)(clk >> 8);
    cmd[9] = (uint8_t)(clk);
    cmd[10] = raF;
    cmd[11] = snF;
    cmd[12] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 13);
}

/**
 * @brief       立即停止
 * @param       addr: 电机地址
 * @param       snF: 多机同步标志
 * @retval      �?
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 0 && addr <= 255);  /* 支持广播地址0 */
    
    uint8_t cmd[5];
    cmd[0] = addr; cmd[1] = 0xFE; cmd[2] = 0x98;
    cmd[3] = snF; cmd[4] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 5);
}

/**
 * @brief       多机同步运动
 * @param       addr: 电机地址
 * @retval      �?
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 0 && addr <= 255);  /* 通常使用广播地址0 */
    
    uint8_t cmd[4];
    cmd[0] = addr; cmd[1] = 0xFF; cmd[2] = 0x66; cmd[3] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 4);
}

/**
 * @brief       设置单圈回零的零点位�?
 * @param       addr: 电机地址
 * @param       svF: 是否存储，false不存储，true存储
 * @retval      �?
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    
    uint8_t cmd[5];
    cmd[0] = addr; cmd[1] = 0x93; cmd[2] = 0x88;
    cmd[3] = svF; cmd[4] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 5);
}

/**
 * @brief       修改回零参数
 * @param       addr: 电机地址
 * @param       svF: 是否存储
 * @param       o_mode: 回零模式�?单圈就近�?单圈方向�?多圈无限位，3多圈有限位）
 * @param       o_dir: 回零方向�?为CW，其余为CCW
 * @param       o_vel: 回零速度 RPM
 * @param       o_tm: 回零超时时间 ms
 * @param       sl_vel: 无限位碰撞检测转�?RPM
 * @param       sl_ma: 无限位碰撞检测电�?mA
 * @param       sl_ms: 无限位碰撞检测时�?ms
 * @param       potF: 上电自动触发回零
 * @retval      �?
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, 
                                  uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, 
                                  uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    CHECK_PARAM_VOID(o_mode <= 3);     /* 回零模式0-3 */
    CHECK_PARAM_VOID(o_vel <= 5000);   /* 回零速度0-5000 RPM */
    CHECK_PARAM_VOID(sl_vel <= 5000);  /* 无限位检测速度0-5000 RPM */
    
    uint8_t cmd[20];
    cmd[0] = addr;
    cmd[1] = 0x4C;
    cmd[2] = 0xAE;
    cmd[3] = svF;
    cmd[4] = o_mode;
    cmd[5] = o_dir;
    cmd[6] = (uint8_t)(o_vel >> 8);
    cmd[7] = (uint8_t)(o_vel);
    cmd[8] = (uint8_t)(o_tm >> 24);
    cmd[9] = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);
    cmd[11] = (uint8_t)(o_tm);
    cmd[12] = (uint8_t)(sl_vel >> 8);
    cmd[13] = (uint8_t)(sl_vel);
    cmd[14] = (uint8_t)(sl_ma >> 8);
    cmd[15] = (uint8_t)(sl_ma);
    cmd[16] = (uint8_t)(sl_ms >> 8);
    cmd[17] = (uint8_t)(sl_ms);
    cmd[18] = potF;
    cmd[19] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 20);
}

/**
 * @brief       触发回零
 * @param       addr: 电机地址
 * @param       o_mode: 回零模式
 * @param       snF: 多机同步标志
 * @retval      �?
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    CHECK_PARAM_VOID(o_mode <= 3);  /* 回零模式0-3 */
    
    uint8_t cmd[5];
    cmd[0] = addr; cmd[1] = 0x9A; cmd[2] = o_mode;
    cmd[3] = snF; cmd[4] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 5);
}

/**
 * @brief       强制中断并退出回�?
 * @param       addr: 电机地址
 * @retval      �?
 */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
    /* V3.5 Phase 3: 参数验证 */
    CHECK_PARAM_VOID(addr >= 1 && addr <= 255);
    
    uint8_t cmd[4];
    cmd[0] = addr; cmd[1] = 0x9C; cmd[2] = 0x48; cmd[3] = EMM_V5_CHECKSUM;
    EMM_V5_SEND_CMD(cmd, 4);
}
