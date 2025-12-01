/**
 ****************************************************************************************************
 * @file        iwdg.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-12-01
 * @brief       独立看门狗(IWDG)驱动实现
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M48Z-M3最小系统板STM32F103版
 *
 * 使用示例:
 *   iwdg_init(IWDG_TIMEOUT_1S);   // 初始化1秒超时
 *   while(1) {
 *       // 执行任务
 *       iwdg_feed();                // 喂狗，必须在1秒内调用
 *   }
 *
 ****************************************************************************************************
 */

#include "iwdg.h"

/* 全局句柄 */
IWDG_HandleTypeDef g_iwdg_handle;

/**
 * @brief       初始化独立看门狗
 * @param       prescaler: 预分频系数
 *              @arg IWDG_PRESCALER_4   : 预分频值4
 *              @arg IWDG_PRESCALER_8   : 预分频值8
 *              @arg IWDG_PRESCALER_16  : 预分频值16
 *              @arg IWDG_PRESCALER_32  : 预分频值32
 *              @arg IWDG_PRESCALER_64  : 预分频值64
 *              @arg IWDG_PRESCALER_128 : 预分频值128
 *              @arg IWDG_PRESCALER_256 : 预分频值256
 * @param       reload: 重载值(0-4095)
 *              超时时间(ms) = (prescaler * reload) / 32
 *              例如: prescaler=64, reload=500 -> 超时时间 = (64*500)/32 = 1000ms
 * @retval      无
 * @note        IWDG时钟来自LSI（约32KHz）
 *              一旦启动后无法关闭，只能通过系统复位停止
 */
void iwdg_init(uint8_t prescaler, uint16_t reload)
{
    /* 配置IWDG参数 */
    g_iwdg_handle.Instance = IWDG;
    g_iwdg_handle.Init.Prescaler = prescaler;
    g_iwdg_handle.Init.Reload = reload;
    
    /* 初始化IWDG */
    HAL_IWDG_Init(&g_iwdg_handle);
}

/**
 * @brief       喂看门狗（重载计数器）
 * @param       无
 * @retval      无
 * @note        必须在超时时间内周期性调用此函数，否则系统复位
 */
void iwdg_feed(void)
{
    HAL_IWDG_Refresh(&g_iwdg_handle);
}
