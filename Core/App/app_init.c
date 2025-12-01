/**
 ****************************************************************************************************
 * @file        app_init.c
 * @author      STM32_485 Project Team
 * @version     V3.0
 * @date        2025-12-01
 * @brief       应用层初始化编排实现（App层）
 ****************************************************************************************************
 */

#include "app_init.h"
#include "gpio.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "error_handler.h"    /* V3.5: 错误处理系统 */
#include "comm_monitor.h"     /* V3.5: 通信监控 */
#include <stdio.h>

/**
 * @brief       系统核心初始化
 * @param       无
 * @retval      BSP_INIT_OK: 成功, 其他: 失败
 * @note        包含：HAL库初始化、系统时钟配置、延时初始化
 */
bsp_init_status_t bsp_system_init(void)
{
    /* HAL库初始化 */
    if (HAL_Init() != HAL_OK)
    {
        return BSP_INIT_PERIPH_ERROR;
    }
    
    /* CubeMX生成的GPIO初始化 */
    MX_GPIO_Init();
    
    /* 配置系统时钟：HSE 8MHz + PLL ×9 = 72MHz */
    sys_stm32_clock_init(RCC_PLL_MUL9);
    
    /* 延时函数初始化（72MHz） */
    delay_init(72);
    
    return BSP_INIT_OK;
}

/**
 * @brief       外设初始化
 * @param       无
 * @retval      BSP_INIT_OK: 成功, 其他: 失败
 * @note        包含：LED、按键、USART1+2
 */
bsp_init_status_t bsp_peripheral_init(void)
{
    /* LED初始化 */
    led_init();
    
    /* 按键初始化 */
    key_init();
    
    /* 双串口初始化 */
    usart_init(115200);  /* USART1+2统一初始化 */
    
    /* V3.5: 错误处理系统初始化 */
    error_handler_init();
    
    /* V3.5: 通信监控初始化 */
    comm_monitor_init();
    
    return BSP_INIT_OK;
}

/**
 * @brief       打印系统启动信息
 * @param       无
 * @retval      无
 * @note        通过USART1输出版本、时钟、编译时间等信息
 */
void bsp_print_boot_info(void)
{
    extern uint32_t SystemCoreClock;
    
    printf("\r\n");
    printf("========================================\r\n");
    printf("  STM32F103 Motor Control System V3.0  \r\n");
    printf("========================================\r\n");
    printf("System Clock:   %lu MHz\r\n", (unsigned long)(SystemCoreClock / 1000000));
    printf("APB1 Clock:     %lu MHz\r\n", (unsigned long)(HAL_RCC_GetPCLK1Freq() / 1000000));
    printf("APB2 Clock:     %lu MHz\r\n", (unsigned long)(HAL_RCC_GetPCLK2Freq() / 1000000));
    printf("Compile Date:   %s %s\r\n", __DATE__, __TIME__);
    printf("Architecture:   V3.0 (3-Layer Design)\r\n");
    printf("========================================\r\n\r\n");
}
