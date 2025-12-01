/**
 ****************************************************************************************************
 * @file        sys_timer.c
 * @author      STM32_485 Project Team
 * @version     V1.0
 * @date        2025-12-02
 * @brief       非阻塞软件定时器实现
 ****************************************************************************************************
 */

#include "sys_timer.h"

/**
 * @brief       启动定时器
 * @param       timer: 定时器指针
 * @param       timeout_ms: 超时时间(ms)
 * @param       auto_reload: 是否自动重载（true=周期触发，false=单次触发）
 * @retval      无
 */
void sys_timer_start(sys_timer_t *timer, uint32_t timeout_ms, bool auto_reload)
{
    if (timer == NULL) return;
    
    timer->start_tick = HAL_GetTick();
    timer->timeout_ms = timeout_ms;
    timer->auto_reload = auto_reload;
    timer->active = true;
}

/**
 * @brief       停止定时器
 * @param       timer: 定时器指针
 * @retval      无
 */
void sys_timer_stop(sys_timer_t *timer)
{
    if (timer == NULL) return;
    
    timer->active = false;
}

/**
 * @brief       检查定时器是否超时
 * @param       timer: 定时器指针
 * @retval      true: 超时, false: 未超时
 * @note        自动重载模式会自动重新启动定时器
 */
bool sys_timer_expired(sys_timer_t *timer)
{
    if (timer == NULL || !timer->active)
    {
        return false;
    }
    
    uint32_t elapsed = HAL_GetTick() - timer->start_tick;
    
    if (elapsed >= timer->timeout_ms)
    {
        if (timer->auto_reload)
        {
            /* 自动重载：更新启动时间戳 */
            timer->start_tick = HAL_GetTick();
        }
        else
        {
            /* 单次触发：停止定时器 */
            timer->active = false;
        }
        return true;
    }
    
    return false;
}

/**
 * @brief       获取定时器剩余时间
 * @param       timer: 定时器指针
 * @retval      剩余时间(ms)，定时器未激活返回0
 */
uint32_t sys_timer_remaining(const sys_timer_t *timer)
{
    if (timer == NULL || !timer->active)
    {
        return 0;
    }
    
    uint32_t elapsed = HAL_GetTick() - timer->start_tick;
    
    if (elapsed >= timer->timeout_ms)
    {
        return 0;
    }
    
    return (timer->timeout_ms - elapsed);
}

/**
 * @brief       检查定时器是否激活
 * @param       timer: 定时器指针
 * @retval      true: 激活, false: 未激活
 */
bool sys_timer_is_active(const sys_timer_t *timer)
{
    if (timer == NULL)
    {
        return false;
    }
    
    return timer->active;
}
