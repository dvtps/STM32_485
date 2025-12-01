/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_RS485模块 测试代码
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

#include "demo.h"
#include "./BSP/ATK_RS485/atk_rs485.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/KEY/key.h"
#include "./BSP/LED/led.h"
#include <stdio.h>


/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t key;
    
    uint8_t i = 0, t = 0;
    uint8_t cnt = 0;
    uint8_t rs485buf[200];
    
    atk_rs485_init(115200);                     /* 初始化RS485，波特率115200 */
    
    while (1)
    {
        key = key_scan(0);

        if (key == KEY0_PRES)                   /* KEY0按下,发送一次数据 */
        {
            printf("RS485模块 发送的数据为：");
            
            for (i = 0; i < 200; i++)
            {
                rs485buf[i] = cnt + i;          /* 填充发送缓冲区 */
                printf("%02X", rs485buf[i]);    /* 打印输出数据 */
            }
            
            printf("\r\n");

            atk_rs485_send_data(rs485buf, 200); /* 发送200个字节 */
        }

        atk_rs485_receive_data(rs485buf, &key);

        if (key)                                /* 接收到数据 */
        {   
            if (key > 200)key = 200;            /* 最大有效字节数：200个 */

            printf("RS485模块 接收的数据为：");
            
            for (i = 0; i < key; i++)
            {
                printf("%02X", rs485buf[i]);    /* 打印输出数据 */
            }
            
            printf("\r\n");
        }

        t++;
        delay_ms(10);

        if (t == 20)
        {
            LED0_TOGGLE();                      /* LED0闪烁, 提示系统正在运行 */
            t = 0;
            cnt++;
        }
    }
}


