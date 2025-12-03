/**
 ****************************************************************************************************
 * @file        tim2_rt.c
 * @author      STM32_485 Project
 * @version     V1.0
 * @date        2025-12-03
 * @brief       TIM2定时器配置（实时电机10kHz控制周期）
 ****************************************************************************************************
 * @attention
 * 
 * 功能说明：
 * - TIM2配置为100μs中断周期（10kHz）
 * - 用于驱动实时电机系统自动处理命令队列
 * - 中断优先级：Preemption=1（低于DMA，高于USART）
 * 
 * 时钟计算：
 * - TIM2时钟源：APB1 × 2 = 36MHz × 2 = 72MHz
 * - Prescaler = 71 → 72MHz / (71+1) = 1MHz
 * - Period = 99 → 1MHz / (99+1) = 10kHz (100μs)
 * 
 ****************************************************************************************************
 */

#include "tim2_rt.h"
#include "stm32f1xx_hal.h"
#include "app_config.h"

#if REALTIME_MOTOR_ENABLE

/* TIM2句柄 */
TIM_HandleTypeDef g_htim2_rt;

/**
 * @brief       TIM2初始化（10kHz控制周期）
 * @param       无
 * @retval      0=成功, 1=失败
 */
uint8_t tim2_rt_init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* TIM2时钟使能 */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2基本配置 */
    g_htim2_rt.Instance = TIM2;
    g_htim2_rt.Init.Prescaler = 71;              /* 72MHz/72 = 1MHz */
    g_htim2_rt.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_htim2_rt.Init.Period = 99;                 /* 1MHz/100 = 10kHz */
    g_htim2_rt.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_htim2_rt.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&g_htim2_rt) != HAL_OK)
    {
        return 1;  /* 初始化失败 */
    }

    /* 时钟源配置 */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&g_htim2_rt, &sClockSourceConfig) != HAL_OK)
    {
        return 1;
    }

    /* 主输出配置（可选） */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_htim2_rt, &sMasterConfig) != HAL_OK)
    {
        return 1;
    }

    /* 配置TIM2中断（优先级1，低于DMA的0） */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return 0;  /* 初始化成功 */
}

/**
 * @brief       启动TIM2定时器
 * @param       无
 * @retval      无
 */
void tim2_rt_start(void)
{
    HAL_TIM_Base_Start_IT(&g_htim2_rt);
}

/**
 * @brief       停止TIM2定时器
 * @param       无
 * @retval      无
 */
void tim2_rt_stop(void)
{
    HAL_TIM_Base_Stop_IT(&g_htim2_rt);
}

/**
 * @brief       获取TIM2运行状态
 * @param       无
 * @retval      0=停止, 1=运行
 */
uint8_t tim2_rt_is_running(void)
{
    return (g_htim2_rt.Instance->CR1 & TIM_CR1_CEN) ? 1 : 0;
}

#endif /* REALTIME_MOTOR_ENABLE */
