/**
 ****************************************************************************************************
 * @file        atk_rs485.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_RS485模块 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M48Z-M3最小系统板STM32F103版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./BSP/ATK_RS485/atk_rs485.h"
#include "./SYSTEM/delay/delay.h"


UART_HandleTypeDef g_rs458_handler;     /* RS485控制句柄(串口) */

uint8_t g_RS485_rx_buf[RS485_REC_LEN];  /* 接收缓冲, 最大RS485_REC_LEN个字节 */
uint8_t g_RS485_rx_cnt = 0;             /* 接收到的数据长度 */


#ifdef RS485_EN_RX                      /* 如果使能了接收 */

/**
 * @brief       RS485相关串口中断服务函数
 * @param       无
 * @retval      无
 */
void ATK_RS485_UX_IRQHandler(void)
{
    uint8_t res;

    if ((__HAL_UART_GET_FLAG(&g_rs458_handler, UART_FLAG_RXNE) != RESET))   /* 接收到数据 */
    {
        HAL_UART_Receive(&g_rs458_handler, &res, 1, 1000);                  /* 取出数据 */

        if (g_RS485_rx_cnt < RS485_REC_LEN)                                 /* 缓冲区未满 */
        {
            g_RS485_rx_buf[g_RS485_rx_cnt] = res;                           /* 记录接收到的值 */
            g_RS485_rx_cnt++;                                               /* 接收数据增加1 */
        }
    }
}

#endif

/**
 * @brief       ATK_RS485模块初始化函数
 *   @note      该函数主要是初始化串口
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void atk_rs485_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    ATK_RS485_TX_GPIO_CLK_ENABLE();                             /* 使能 串口TX脚 时钟 */
    ATK_RS485_RX_GPIO_CLK_ENABLE();                             /* 使能 串口RX脚 时钟 */
    ATK_RS485_UX_CLK_ENABLE();                                  /* 使能 串口 时钟 */

    gpio_init_struct.Pin = ATK_RS485_TX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ATK_RS485_TX_GPIO_PORT, &gpio_init_struct);   /* 串口TX引脚初始化 */

    gpio_init_struct.Pin = ATK_RS485_RX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(ATK_RS485_RX_GPIO_PORT, &gpio_init_struct);   /* 串口RX引脚初始化 */

    /* USART 初始化设置 */
    g_rs458_handler.Instance = ATK_RS485_UX;                    /* 选择485对应的串口 */
    g_rs458_handler.Init.BaudRate = baudrate;                   /* 波特率 */
    g_rs458_handler.Init.WordLength = UART_WORDLENGTH_8B;       /* 字长为8位数据格式 */
    g_rs458_handler.Init.StopBits = UART_STOPBITS_1;            /* 一个停止位 */
    g_rs458_handler.Init.Parity = UART_PARITY_NONE;             /* 无奇偶校验位 */
    g_rs458_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;       /* 无硬件流控 */
    g_rs458_handler.Init.Mode = UART_MODE_TX_RX;                /* 收发模式 */
    HAL_UART_Init(&g_rs458_handler);                            /* HAL_UART_Init()会使能UART2 */

#if RS485_EN_RX         /* 如果使能了接收 */

    __HAL_UART_ENABLE_IT(&g_rs458_handler, UART_IT_RXNE);       /* 开启接收中断 */
    HAL_NVIC_EnableIRQ(ATK_RS485_UX_IRQn);                      /* 使能USART2中断 */
    HAL_NVIC_SetPriority(ATK_RS485_UX_IRQn, 3, 3);              /* 抢占优先级3，子优先级3 */

#endif
}

/**
 * @brief       RS485发送len个字节数据
 * @param       buf     : 缓冲区指针
 * @param       len     : 发送的字节数(为了和本代码的接收匹配,这里建议不要超过 RS485_REC_LEN 个字节)
 * @retval      无
 */
void atk_rs485_send_data(uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit(&g_rs458_handler, buf, len, 1000);        /* 串口发送数据 */
    g_RS485_rx_cnt = 0;                                         /* 清空计数 */
}

/**
 * @brief       RS485查询接收到的数据
 * @param       buf : 接收缓冲区首地址
 * @param       len : 接收到的数据长度
 *   @arg             0,表示没有接收到任何数据
 *   @arg             其他, 表示接收到的数据长度
 * @retval      无
 */
void atk_rs485_receive_data(uint8_t *buf, uint8_t *len)
{
    uint8_t rxlen = g_RS485_rx_cnt;
    uint8_t i = 0;
    *len = 0;                               /* 默认为0 */
    
    delay_ms(5);                            /* 等待5ms,连续超过5ms没有接收到一个数据,则认为接收结束 */

    if (rxlen == g_RS485_rx_cnt && rxlen)   /* 接收到了数据,且接收完成了 */
    {
        for (i = 0; i < rxlen; i++)
        {
            buf[i] = g_RS485_rx_buf[i];     /* 取出数据，保存到buf中 */
        }

        *len = g_RS485_rx_cnt;              /* 记录本次数据长度 */
        g_RS485_rx_cnt = 0;                 /* 清零 */
    }
}


