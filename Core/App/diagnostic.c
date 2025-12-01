/**
 ****************************************************************************************************
 * @file        diagnostic.c
 * @author      STM32_485 Project Team
 * @version     V3.0
 * @date        2025-12-01
 * @brief       系统诊断模块实现
 ****************************************************************************************************
 */

#include "diagnostic.h"
#include "led.h"
#include "usart.h"
#include <stdio.h>

/**
 * @brief       运行完整诊断流程
 * @param       无
 * @retval      无
 * @note        包含时钟检测、USART检测、LED闪烁显示
 */
void diag_run_full_check(void)
{
    diag_blink_clock_speed();
    HAL_Delay(1000);
    
    diag_print_clock_info();
    diag_print_usart_config();
    
    printf(">>> Diagnostics Complete <<<\r\n\r\n");
}

/**
 * @brief       打印时钟信息
 * @param       无
 * @retval      无
 */
void diag_print_clock_info(void)
{
    extern uint32_t SystemCoreClock;
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    
    printf("----------------------------------------\r\n");
    printf("Clock Diagnostic:\r\n");
    printf("  SystemCoreClock:  %lu Hz (expect 72MHz)\r\n", (unsigned long)SystemCoreClock);
    printf("  APB1 (USART2):    %lu Hz (expect 36MHz)\r\n", (unsigned long)pclk1);
    printf("  APB2 (USART1):    %lu Hz (expect 72MHz)\r\n", (unsigned long)pclk2);
    printf("----------------------------------------\r\n");
}

/**
 * @brief       打印USART配置信息
 * @param       无
 * @retval      无
 */
void diag_print_usart_config(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    uint32_t usart1_brr = USART1->BRR;
    uint32_t usart2_brr = USART2->BRR;
    
    printf("USART Diagnostic:\r\n");
    printf("  USART1 (PA9/PA10) - Debug Output:\r\n");
    printf("    BRR:         0x%04lX (%lu)\r\n", (unsigned long)usart1_brr, (unsigned long)usart1_brr);
    printf("    Actual Baud: %lu bps (target 115200)\r\n", (unsigned long)(pclk2 / usart1_brr));
    printf("  USART2 (PA2/PA3)  - RS485 Motor:\r\n");
    printf("    BRR:         0x%04lX (%lu)\r\n", (unsigned long)usart2_brr, (unsigned long)usart2_brr);
    printf("    Actual Baud: %lu bps (target 115200)\r\n", (unsigned long)(pclk1 / usart2_brr));
    printf("----------------------------------------\r\n");
}

/**
 * @brief       用LED闪烁显示系统时钟频率
 * @param       无
 * @retval      无
 * @note        闪烁次数 = SystemCoreClock / 1MHz（72MHz闪72次）
 */
void diag_blink_clock_speed(void)
{
    extern uint32_t SystemCoreClock;
    uint8_t blink_count = SystemCoreClock / 1000000;
    
    for (uint8_t i = 0; i < blink_count; i++)
    {
        LED0(0);  /* LED亮 */
        HAL_Delay(50);
        LED0(1);  /* LED灭 */
        HAL_Delay(50);
    }
}
