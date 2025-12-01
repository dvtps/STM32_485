/**
 ****************************************************************************************************
 * @file        main_rs485_test.c
 * @brief       USART2 RS485通信测试程序 (简化版)
 * @date        2025-12-01
 ****************************************************************************************************
 * 功能说明:
 * - USART1 (PA9/PA10):  115200bps，用于printf调试输出
 * - USART2 (PA2/PA3):   115200bps，用于RS485通信（ATK-MB024模块）
 * 
 * 测试步骤:
 * 1. 将本文件内容复制替换Core/App/main.c的主要部分
 * 2. 编译烧录
 * 3. PA9/PA10连接USB-TTL查看调试信息（115200-8-N-1）
 * 4. PA2/PA3通过RS485模块连接串口助手（115200-8-N-1，编码GBK）
 * 5. STM32每秒自动发送测试消息
 * 6. 串口助手发送数据，STM32会回显
 ****************************************************************************************************
 */

#include "main.h"
#include "gpio.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
    /* 系统初始化 */
    HAL_Init();
    MX_GPIO_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);     /* HSE 8MHz + PLL ×9 = 72MHz */
    delay_init(72);                          /* 72MHz系统时钟 */
    led_init();
    
    /* 初始化串口 */
    usart1_init(115200);                     /* USART1: 调试输出 (PA9/PA10) */
    usart2_init(115200);                     /* USART2: RS485通信 (PA2/PA3) */
    
    /* 诊断：读取实际时钟频率和USART寄存器 */
    extern uint32_t SystemCoreClock;
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();  /* APB1时钟 (USART2挂在APB1) */
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();  /* APB2时钟 (USART1挂在APB2) */
    uint32_t usart1_brr = USART1->BRR;        /* USART1波特率寄存器 */
    uint32_t usart2_brr = USART2->BRR;        /* USART2波特率寄存器 */
    
    /* 构造诊断报告字符串 */
    char diag_report[800];
    snprintf(diag_report, sizeof(diag_report),
        "\r\n========================================\r\n"
        "  STM32F103 Dual USART Configuration\r\n"
        "========================================\r\n"
        "Clock Source:     HSE 8MHz + PLL ×9\r\n"
        "SystemCoreClock:  %lu Hz (expect 72MHz)\r\n"
        "APB1 (USART2):    %lu Hz (expect 36MHz)\r\n"
        "APB2 (USART1):    %lu Hz (expect 72MHz)\r\n"
        "----------------------------------------\r\n"
        "USART1 (PA9/PA10) - CMSIS-DAP Virtual COM\r\n"
        "  BRR:            0x%04lX (%lu, expect 0x0271=625)\r\n"
        "  Actual Baud:    %lu bps (target 115200)\r\n"
        "  Hardware:       ATK-CMSIS-DAP -> COM5\r\n"
        "----------------------------------------\r\n"
        "USART2 (PA2/PA3)  - Active (printf output)\r\n"
        "  BRR:            0x%04lX (%lu, expect 0x0138=312)\r\n"
        "  Actual Baud:    %lu bps (target 115200)\r\n"
        "  Hardware:       ATK-MB024 RS485 -> CH340 -> COM7\r\n"
        "========================================\r\n\r\n",
        (unsigned long)SystemCoreClock,
        (unsigned long)pclk1,
        (unsigned long)pclk2,
        (unsigned long)usart1_brr, (unsigned long)usart1_brr,
        (unsigned long)(pclk2 / usart1_brr),
        (unsigned long)usart2_brr, (unsigned long)usart2_brr,
        (unsigned long)(pclk1 / usart2_brr)
    );
    
    /* LED闪烁显示SystemCoreClock（用于硬件调试）*/
    /* 闪烁次数 = SystemCoreClock / 1000000（每MHz闪1次）*/
    uint8_t blink_count = SystemCoreClock / 1000000;
    for (uint8_t i = 0; i < blink_count; i++)
    {
        LED0(0);  /* LED亮 */
        HAL_Delay(200);
        LED0(1);  /* LED灭 */
        HAL_Delay(200);
    }
    HAL_Delay(1000);  /* 暂停1秒 */
    
    /* 测试USART1硬件：直接用HAL发送 */
    const char* hal_test = "\r\n[HAL TEST] USART1 Hardware Check\r\n";
    HAL_UART_Transmit(&g_uart1_handle, (uint8_t*)hal_test, strlen(hal_test), 1000);
    
    /* 输出诊断报告到USART1 (COM5 CMSIS-DAP虚拟串口) */
    printf("%s", diag_report);
    printf(">>> Diagnostics Complete <<<\r\n");
    printf(">>> COM5: Debug output (USART1)\r\n");
    printf(">>> COM7: RS485 test (USART2)\r\n\r\n");
    
    /* 再次HAL测试确认 */
    const char* hal_test2 = "[HAL TEST] After printf\r\n\r\n";
    HAL_UART_Transmit(&g_uart1_handle, (uint8_t*)hal_test2, strlen(hal_test2), 1000);
    
    uint32_t test_counter = 0;
    
    while (1)
    {
        /* RS485测试：通过USART2发送测试字符 (COM7) */
        HAL_Delay(500);  /* 0.5秒间隔 */
        
        /* 方法1：通过HAL_UART_Transmit发送 */
        const char test_char = 'U';  /* 0x55，波形方便示波器观察 */
        HAL_UART_Transmit(&g_uart2_handle, (uint8_t*)&test_char, 1, 100);
        
        /* 方法2：直接写USART2寄存器（绕过HAL库） */
        while (!(USART2->SR & USART_SR_TXE));  /* 等待发送缓冲区空 */
        USART2->DR = 'A';  /* 直接写数据寄存器 */
        
        /* LED闪烁指示 */
        LED0_TOGGLE();
        
        test_counter++;
        
        /* 检查USART2是否接收到数据 */
        if (g_emm_frame_complete)
        {
            g_emm_frame_complete = 0;
            
            /* 调试信息输出到USART1 */
            printf("[RX] USART2 received %d bytes: ", g_emm_rx_count);
            for (int i = 0; i < g_emm_rx_count && i < 32; i++)
            {
                if (g_emm_rx_cmd[i] >= 0x20 && g_emm_rx_cmd[i] <= 0x7E)
                {
                    printf("%c", g_emm_rx_cmd[i]);  /* 可打印字符 */
                }
                else
                {
                    printf("[%02X]", g_emm_rx_cmd[i]);  /* 非打印字符用十六进制 */
                }
            }
            printf("\r\n");
            
            /* 回显接收到的数据（通过USART2） */
            const char *echo_prefix = "\r\n[ECHO] ";
            HAL_UART_Transmit(&g_uart2_handle, (uint8_t*)echo_prefix, strlen(echo_prefix), 10);
            HAL_UART_Transmit(&g_uart2_handle, g_emm_rx_cmd, g_emm_rx_count, 100);
            HAL_UART_Transmit(&g_uart2_handle, (uint8_t*)"\r\n\r\n", 4, 10);
            
            LED0_TOGGLE();  /* LED闪烁指示接收 */
        }
        
        delay_ms(10);
    }
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        LED0_TOGGLE();
        HAL_Delay(200);
    }
}
