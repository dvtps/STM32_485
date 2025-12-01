/**
 ****************************************************************************************************
 * @file        key.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       按键输入 驱动代码
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

#include "key.h"
#include "stm32f1xx_hal.h"

/* 按键状态机定义 */
typedef enum {
    KEY_STATE_IDLE = 0,        /* 空闲状态 */
    KEY_STATE_DEBOUNCE,        /* 去抖动状态 */
    KEY_STATE_PRESSED          /* 按下确认状态 */
} key_state_t;

/* 私有变量 */
static key_state_t key0_state = KEY_STATE_IDLE;
static key_state_t wkup_state = KEY_STATE_IDLE;
static uint32_t key0_debounce_tick = 0;
static uint32_t wkup_debounce_tick = 0;

#define KEY_DEBOUNCE_TIME_MS    10    /* 去抖动时间10ms */

/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    KEY0_GPIO_CLK_ENABLE();                                     /* KEY0时钟使能 */
    WKUP_GPIO_CLK_ENABLE();                                     /* WKUP时钟使能 */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = WKUP_GPIO_PIN;                       /* WKUP引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                      /* 下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP引脚模式设置,下拉输入 */
}

/**
 * @brief       按键扫描函数（状态机实现，非阻塞）
 * @note        优先级: WK_UP > KEY0
 * @param       mode:0 / 1,具体含义如下:
 *   @arg       0,  不支持连续按
 *   @arg       1,  支持连续按
 * @retval      键值: KEY0_PRES(1) / WKUP_PRES(2) / 0(无按键)
 */
uint8_t key_scan(uint8_t mode)
{
    uint8_t key_val = 0;
    uint32_t current_tick = HAL_GetTick();
    
    /* 状态机处理KEY0 */
    switch (key0_state)
    {
        case KEY_STATE_IDLE:
            if (KEY0 == 1)  /* 检测到低电平(按下) */
            {
                key0_state = KEY_STATE_DEBOUNCE;
                key0_debounce_tick = current_tick;
            }
            break;
            
        case KEY_STATE_DEBOUNCE:
            if ((current_tick - key0_debounce_tick) >= KEY_DEBOUNCE_TIME_MS)
            {
                if (KEY0 == 1)  /* 去抖后仍为低电平 */
                {
                    key0_state = KEY_STATE_PRESSED;
                    key_val = KEY0_PRES;
                }
                else  /* 抖动干扰，回到空闲 */
                {
                    key0_state = KEY_STATE_IDLE;
                }
            }
            break;
            
        case KEY_STATE_PRESSED:
            if (mode == 1)  /* 支持连按 */
            {
                key_val = KEY0_PRES;
            }
            if (KEY0 == 0)  /* 按键释放 */
            {
                key0_state = KEY_STATE_IDLE;
            }
            break;
    }
    
    /* 状态机处理WKUP（高优先级，可覆盖KEY0） */
    switch (wkup_state)
    {
        case KEY_STATE_IDLE:
            if (WK_UP == 1)  /* 检测到高电平(按下) */
            {
                wkup_state = KEY_STATE_DEBOUNCE;
                wkup_debounce_tick = current_tick;
            }
            break;
            
        case KEY_STATE_DEBOUNCE:
            if ((current_tick - wkup_debounce_tick) >= KEY_DEBOUNCE_TIME_MS)
            {
                if (WK_UP == 1)  /* 去抖后仍为高电平 */
                {
                    wkup_state = KEY_STATE_PRESSED;
                    key_val = WKUP_PRES;  /* 覆盖KEY0 */
                }
                else  /* 抖动干扰，回到空闲 */
                {
                    wkup_state = KEY_STATE_IDLE;
                }
            }
            break;
            
        case KEY_STATE_PRESSED:
            if (mode == 1)  /* 支持连按 */
            {
                key_val = WKUP_PRES;
            }
            if (WK_UP == 0)  /* 按键释放 */
            {
                wkup_state = KEY_STATE_IDLE;
            }
            break;
    }
    
    return key_val;
}
