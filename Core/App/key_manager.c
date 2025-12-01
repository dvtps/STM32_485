/**
 ****************************************************************************************************
 * @file        key_manager.c
 * @author      STM32_485 Project
 * @version     V3.2
 * @date        2025-12-02
 * @brief       按键管理器（App层：去抖+状态机）
 ****************************************************************************************************
 */

#include "key_manager.h"
#include "key.h"
#include "stm32f1xx_hal.h"

/* 私有状态变量（App层业务状态） */
static key_manager_state_t key0_state = KEY_MGR_STATE_IDLE;
static key_manager_state_t wkup_state = KEY_MGR_STATE_IDLE;
static uint32_t key0_debounce_tick = 0;
static uint32_t wkup_debounce_tick = 0;

/**
 * @brief       初始化按键管理器
 * @param       无
 * @retval      无
 */
void key_manager_init(void)
{
    key_init();  /* 调用BSP层GPIO初始化 */
    key_manager_reset();
}

/**
 * @brief       重置状态机
 * @param       无
 * @retval      无
 */
void key_manager_reset(void)
{
    key0_state = KEY_MGR_STATE_IDLE;
    wkup_state = KEY_MGR_STATE_IDLE;
    key0_debounce_tick = 0;
    wkup_debounce_tick = 0;
}

/**
 * @brief       按键扫描函数（状态机实现，非阻塞）
 * @note        优先级: WKUP > KEY0
 * @param       mode: 0=不支持连按, 1=支持连按
 * @retval      键值: KEY_MGR_KEY0_PRESS(1) / KEY_MGR_WKUP_PRESS(2) / 0(无按键)
 */
uint8_t key_manager_scan(uint8_t mode)
{
    uint8_t key_val = 0;
    uint32_t current_tick = HAL_GetTick();
    
    /* 状态机处理KEY0 */
    switch (key0_state)
    {
        case KEY_MGR_STATE_IDLE:
            if (key_read_key0())  /* 调用BSP层读取原始GPIO */
            {
                key0_state = KEY_MGR_STATE_DEBOUNCE;
                key0_debounce_tick = current_tick;
            }
            break;
            
        case KEY_MGR_STATE_DEBOUNCE:
            if ((current_tick - key0_debounce_tick) >= KEY_MGR_DEBOUNCE_MS)
            {
                if (key_read_key0())  /* 去抖后仍按下 */
                {
                    key0_state = KEY_MGR_STATE_PRESSED;
                    key_val = KEY_MGR_KEY0_PRESS;
                }
                else  /* 抖动干扰，回到空闲 */
                {
                    key0_state = KEY_MGR_STATE_IDLE;
                }
            }
            break;
            
        case KEY_MGR_STATE_PRESSED:
            if (mode == 1)  /* 支持连按 */
            {
                key_val = KEY_MGR_KEY0_PRESS;
            }
            if (!key_read_key0())  /* 按键释放 */
            {
                key0_state = KEY_MGR_STATE_IDLE;
            }
            break;
    }
    
    /* 状态机处理WKUP（高优先级，可覆盖KEY0） */
    switch (wkup_state)
    {
        case KEY_MGR_STATE_IDLE:
            if (key_read_wkup())  /* 调用BSP层读取原始GPIO */
            {
                wkup_state = KEY_MGR_STATE_DEBOUNCE;
                wkup_debounce_tick = current_tick;
            }
            break;
            
        case KEY_MGR_STATE_DEBOUNCE:
            if ((current_tick - wkup_debounce_tick) >= KEY_MGR_DEBOUNCE_MS)
            {
                if (key_read_wkup())  /* 去抖后仍按下 */
                {
                    wkup_state = KEY_MGR_STATE_PRESSED;
                    key_val = KEY_MGR_WKUP_PRESS;  /* 覆盖KEY0 */
                }
                else  /* 抖动干扰，回到空闲 */
                {
                    wkup_state = KEY_MGR_STATE_IDLE;
                }
            }
            break;
            
        case KEY_MGR_STATE_PRESSED:
            if (mode == 1)  /* 支持连按 */
            {
                key_val = KEY_MGR_WKUP_PRESS;
            }
            if (!key_read_wkup())  /* 按键释放 */
            {
                wkup_state = KEY_MGR_STATE_IDLE;
            }
            break;
    }
    
    return key_val;
}

