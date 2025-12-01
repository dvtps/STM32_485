/**
 ****************************************************************************************************
 * @file        iwdg_timer.c
 * @author      正点原子团队(ALIENTEK) - Phase 4高级优化
 * @version     V1.0
 * @date        2025-12-01
 * @brief       基于TIM3的定时看门狗喂养模块
 ****************************************************************************************************
 * @attention
 * 
 * 功能说明:
 * - TIM3每500ms触发一次中断
 * - 中断中检查系统健康度，通过后自动喂狗
 * - 避免主循环卡死后仍持续喂狗的问题
 * 
 * 健康检查项:
 * - FIFO水位 < 90%
 * - 电机响应超时计数未异常增长
 * - 主循环心跳标志定期刷新
 * 
 ****************************************************************************************************
 */

#include "iwdg_timer.h"
#include "iwdg.h"
#include "emm_fifo.h"
#include "stm32f1xx_hal.h"

/* TIM3句柄 */
static TIM_HandleTypeDef g_tim3_handle;

/* 健康监控变量 */
static volatile uint32_t g_main_loop_heartbeat = 0;  /* 主循环心跳计数 */
static uint32_t g_last_heartbeat = 0;                /* 上次心跳值 */
static uint32_t g_health_check_fail_count = 0;       /* 健康检查失败次数 */

/**
 * @brief       TIM3初始化（500ms定时中断）
 * @param       无
 * @retval      无
 */
void tim3_watchdog_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();  /* 使能TIM3时钟 */
    
    /* TIM3配置: 72MHz / (7200 * 5000) = 2Hz (500ms周期) */
    g_tim3_handle.Instance = TIM3;
    g_tim3_handle.Init.Prescaler = 7200 - 1;        /* 预分频: 72MHz/7200=10kHz */
    g_tim3_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim3_handle.Init.Period = 5000 - 1;           /* 计数: 10kHz/5000=2Hz */
    g_tim3_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_tim3_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    HAL_TIM_Base_Init(&g_tim3_handle);
    
    /* 配置TIM3中断 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);  /* 优先级2（电机>TIM3>调试） */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    
    /* 启动定时器并使能中断 */
    HAL_TIM_Base_Start_IT(&g_tim3_handle);
}

/**
 * @brief       系统健康度检查
 * @param       无
 * @retval      true: 健康, false: 异常
 */
static bool system_health_check(void)
{
    bool is_healthy = true;
    
    /* 检查1: FIFO水位 */
    uint8_t fifo_usage = emm_fifo_get_usage_percent();
    if (fifo_usage > 90)
    {
        is_healthy = false;
        /* 可选: 记录日志 */
    }
    
    /* 检查2: 主循环心跳（必须在500ms内更新） */
    if (g_main_loop_heartbeat == g_last_heartbeat)
    {
        is_healthy = false;
        /* 主循环可能卡死 */
    }
    g_last_heartbeat = g_main_loop_heartbeat;
    
    /* 检查3: UART发送错误（已迁移至emm_uart模块） */
    #include "emm_uart.h"
    static emm_uart_stats_t last_stats = {0};
    emm_uart_stats_t current_stats;
    emm_uart_get_stats(&current_stats);
    
    if (current_stats.tx_error_cnt - last_stats.tx_error_cnt > 10)  /* 500ms内超过10次错误 */
    {
        is_healthy = false;
    }
    last_stats = current_stats;
    
    return is_healthy;
}

/**
 * @brief       TIM3中断服务函数
 * @param       无
 * @retval      无
 */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&g_tim3_handle, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_FLAG(&g_tim3_handle, TIM_FLAG_UPDATE);
        
        /* 健康检查 */
        if (system_health_check())
        {
            /* 系统健康，喂狗 */
            iwdg_feed();
            g_health_check_fail_count = 0;
        }
        else
        {
            /* 健康检查失败，停止喂狗，让看门狗复位 */
            g_health_check_fail_count++;
            /* 可选: 记录最后状态到FLASH或备份寄存器 */
        }
    }
}

/**
 * @brief       主循环心跳更新（在motor_zdt_run中调用）
 * @param       无
 * @retval      无
 */
void main_loop_heartbeat_update(void)
{
    g_main_loop_heartbeat++;
}

/**
 * @brief       获取健康检查失败次数
 * @param       无
 * @retval      失败次数
 */
uint32_t get_health_check_fail_count(void)
{
    return g_health_check_fail_count;
}
