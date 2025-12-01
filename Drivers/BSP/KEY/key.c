/**
 ****************************************************************************************************
 * @file        key.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V3.2 (架构重构版)
 * @date        2025-12-02
 * @brief       按键输入 驱动代码（BSP层：仅GPIO读取）
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 架构定位：BSP层 - 硬件抽象（无状态）
 * - V3.2移除状态机逻辑 → 迁移到App层key_manager
 * - BSP层仅提供GPIO原始电平读取接口
 * - 去抖/连按/状态管理由上层负责
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
 * @brief       读取KEY0原始电平（BSP层接口）
 * @param       无
 * @retval      0: 未按下（高电平）, 1: 按下（低电平）
 */
uint8_t key_read_key0(void)
{
    return (KEY0 == 1) ? 1 : 0;
}

/**
 * @brief       读取WKUP原始电平（BSP层接口）
 * @param       无
 * @retval      0: 未按下（低电平）, 1: 按下（高电平）
 */
uint8_t key_read_wkup(void)
{
    return (WK_UP == 1) ? 1 : 0;
}

/**
 * @brief       按键扫描函数（兼容旧版，保留向后兼容）
 * @note        ⚠️ V3.2架构重构：建议使用key_read_key0()/key_read_wkup() + App层状态机
 *              此函数简化为立即检测模式，无去抖/无状态
 * @param       mode: 保留参数（兼容性），当前版本无效
 * @retval      键值: KEY0_PRES(1) / WKUP_PRES(2) / 0(无按键)
 */
uint8_t key_scan(uint8_t mode)
{
    (void)mode;  /* 未使用参数 */
    
    /* 简化实现：优先级 WKUP > KEY0 */
    if (WK_UP == 1)
    {
        return WKUP_PRES;
    }
    else if (KEY0 == 1)
    {
        return KEY0_PRES;
    }
    
    return 0;
}
