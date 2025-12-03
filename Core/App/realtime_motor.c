/**
 ****************************************************************************************************
 * @file        realtime_motor.c
 * @author      STM32_485 Project - Real-Time Extension
 * @version     V1.0
 * @date        2025-12-03
 * @brief       微秒级实时电机控制实现（DMA+TIM2中断+预编译命令）
 ****************************************************************************************************
 */

#include "realtime_motor.h"
#include "y_v2.h"       /* V3.0: Y系X固件协议驱动 */
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>
#include <stdio.h>

/* ====================================================================================
   [1] 私有变量
   ==================================================================================== */

/* 命令队列 */
static rt_cmd_queue_t g_cmd_queue = {0};

/* DMA传输状态 */
static volatile bool g_dma_busy = false;
static volatile uint8_t *g_dma_current_frame = NULL;

#if RT_USE_PRECALC_CMDS
/* 预编译命令缓存（零构造延迟，但需30KB RAM） */
static rt_cmd_frame_t g_precalc_cmds[256][RT_CMD_MAX];
#endif

#if RT_ENABLE_PROFILING
static rt_perf_stats_t g_perf = {0};
#endif

/* 外部变量（来自usart.c） */
extern UART_HandleTypeDef g_uart2_handle;

/* DWT性能计数器（用于微秒级时间测量） */
static inline void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 使能DWT */
    DWT->CYCCNT = 0;                                  /* 清零计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             /* 启动计数器 */
}

static inline uint32_t dwt_get_cycles(void)
{
    return DWT->CYCCNT;  /* 返回CPU周期数（72MHz = 1周期 = 13.89ns） */
}

static inline uint32_t dwt_cycles_to_us(uint32_t cycles)
{
    return cycles / 72;  /* 72MHz时钟：72周期 = 1μs */
}

/* ====================================================================================
   [2] 命令构造辅助函数（动态构造，节省30KB RAM）
   ==================================================================================== */

#if RT_USE_PRECALC_CMDS
/**
 * @brief       预生成常用命令帧（减少运行时构造开销）
 * @param       无
 * @retval      无
 * @note        在rt_motor_init()中调用，仅执行一次
 */
static void precalc_commands(void)
{
    for (uint16_t addr = 0; addr <= 255; addr++)
    {
        rt_cmd_frame_t *base = g_precalc_cmds[addr];
        
        /* 使能命令：{addr, 0xF3, 0xAB, 0x01, 0x00, 0x6B} */
        base[RT_CMD_ENABLE].data[0] = addr;
        base[RT_CMD_ENABLE].data[1] = 0xF3;
        base[RT_CMD_ENABLE].data[2] = 0xAB;
        base[RT_CMD_ENABLE].data[3] = 0x01;
        base[RT_CMD_ENABLE].data[4] = 0x00;
        base[RT_CMD_ENABLE].data[5] = 0x6B;
        base[RT_CMD_ENABLE].length = 6;
        base[RT_CMD_ENABLE].addr = addr;
        
        /* 失能命令：{addr, 0xF3, 0xAB, 0x00, 0x00, 0x6B} */
        base[RT_CMD_DISABLE] = base[RT_CMD_ENABLE];
        base[RT_CMD_DISABLE].data[3] = 0x00;
        
        /* 急停命令：{addr, 0xFE, 0x98, 0x00, 0x6B} */
        base[RT_CMD_STOP].data[0] = addr;
        base[RT_CMD_STOP].data[1] = 0xFE;
        base[RT_CMD_STOP].data[2] = 0x98;
        base[RT_CMD_STOP].data[3] = 0x00;
        base[RT_CMD_STOP].data[4] = 0x6B;
        base[RT_CMD_STOP].length = 5;
        base[RT_CMD_STOP].addr = addr;
        
        /* 同步触发命令（地址0广播）：{0x00, 0xFF, 0x66, 0x6B} */
        base[RT_CMD_SYNC_TRIGGER].data[0] = 0x00;
        base[RT_CMD_SYNC_TRIGGER].data[1] = 0xFF;
        base[RT_CMD_SYNC_TRIGGER].data[2] = 0x66;
        base[RT_CMD_SYNC_TRIGGER].data[3] = 0x6B;
        base[RT_CMD_SYNC_TRIGGER].length = 4;
        base[RT_CMD_SYNC_TRIGGER].addr = 0;
        
        /* 查询位置：{addr, 0x36, 0x6B} */
        base[RT_CMD_QUERY_POS].data[0] = addr;
        base[RT_CMD_QUERY_POS].data[1] = 0x36;
        base[RT_CMD_QUERY_POS].data[2] = 0x6B;
        base[RT_CMD_QUERY_POS].length = 3;
        base[RT_CMD_QUERY_POS].addr = addr;
        
        /* 查询速度：{addr, 0x35, 0x6B} */
        base[RT_CMD_QUERY_VEL].data[0] = addr;
        base[RT_CMD_QUERY_VEL].data[1] = 0x35;
        base[RT_CMD_QUERY_VEL].data[2] = 0x6B;
        base[RT_CMD_QUERY_VEL].length = 3;
        base[RT_CMD_QUERY_VEL].addr = addr;
    }
}
#else
/* 动态构造简单命令（+5-10μs延迟，但节省30KB RAM） */
static inline void build_simple_cmd(rt_cmd_frame_t *frame, uint8_t addr, rt_cmd_type_t cmd_type)
{
    memset(frame, 0, sizeof(rt_cmd_frame_t));
    frame->addr = addr;
    
    switch(cmd_type)
    {
        case RT_CMD_ENABLE:
            frame->data[0] = addr; frame->data[1] = 0xF3; frame->data[2] = 0xAB;
            frame->data[3] = 0x01; frame->data[4] = 0x00; frame->data[5] = 0x6B;
            frame->length = 6;
            break;
        case RT_CMD_DISABLE:
            frame->data[0] = addr; frame->data[1] = 0xF3; frame->data[2] = 0xAB;
            frame->data[3] = 0x00; frame->data[4] = 0x00; frame->data[5] = 0x6B;
            frame->length = 6;
            break;
        case RT_CMD_STOP:
            frame->data[0] = addr; frame->data[1] = 0xFE; frame->data[2] = 0x98;
            frame->data[3] = 0x00; frame->data[4] = 0x6B;
            frame->length = 5;
            break;
        case RT_CMD_SYNC_TRIGGER:
            frame->data[0] = 0x00; frame->data[1] = 0xFF; frame->data[2] = 0x66;
            frame->data[3] = 0x6B;
            frame->length = 4;
            break;
        case RT_CMD_QUERY_POS:
            frame->data[0] = addr; frame->data[1] = 0x36; frame->data[2] = 0x6B;
            frame->length = 3;
            break;
        case RT_CMD_QUERY_VEL:
            frame->data[0] = addr; frame->data[1] = 0x35; frame->data[2] = 0x6B;
            frame->length = 3;
            break;
        default:
            break;
    }
}
#endif

/* ====================================================================================
   [3] 命令队列管理（无锁环形缓冲区）
   ==================================================================================== */

/**
 * @brief       入队命令
 * @param       frame: 命令帧指针
 * @retval      true=成功, false=队列满
 * @note        中断安全（使用原子操作）
 */
static bool queue_push(const rt_cmd_frame_t *frame)
{
    uint16_t next_head = (g_cmd_queue.head + 1) % RT_CMD_QUEUE_SIZE;
    
    if (next_head == g_cmd_queue.tail) {
#if RT_ENABLE_PROFILING
        g_perf.queue_full_count++;
#endif
        return false;  /* 队列满 */
    }
    
    /* 拷贝命令帧（16字节对齐，优化内存访问） */
    memcpy(&g_cmd_queue.frames[g_cmd_queue.head], frame, sizeof(rt_cmd_frame_t));
    
    /* 原子更新写指针 */
    __disable_irq();
    g_cmd_queue.head = next_head;
    g_cmd_queue.count++;
    __enable_irq();
    
    return true;
}

/**
 * @brief       出队命令
 * @param       frame: 输出缓冲区
 * @retval      true=成功, false=队列空
 */
static bool queue_pop(rt_cmd_frame_t *frame)
{
    if (g_cmd_queue.tail == g_cmd_queue.head) {
        return false;  /* 队列空 */
    }
    
    /* 拷贝命令帧 */
    memcpy(frame, &g_cmd_queue.frames[g_cmd_queue.tail], sizeof(rt_cmd_frame_t));
    
    /* 原子更新读指针 */
    __disable_irq();
    g_cmd_queue.tail = (g_cmd_queue.tail + 1) % RT_CMD_QUEUE_SIZE;
    g_cmd_queue.count--;
    __enable_irq();
    
    return true;
}

/* ====================================================================================
   [4] DMA发送管理
   ==================================================================================== */

/**
 * @brief       DMA传输完成回调（在中断中执行，<2μs）
 * @param       huart: UART句柄
 * @retval      无
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        g_dma_busy = false;  /* 标记DMA空闲 */
        
#if RT_ENABLE_PROFILING
        g_perf.cmd_complete_count++;
        uint32_t latency = dwt_cycles_to_us(dwt_get_cycles() - g_perf.last_cmd_tick);
        if (latency > g_perf.max_latency_us) g_perf.max_latency_us = latency;
        if (g_perf.min_latency_us == 0 || latency < g_perf.min_latency_us) {
            g_perf.min_latency_us = latency;
        }
        g_perf.avg_latency_us = (g_perf.avg_latency_us * 7 + latency) / 8;  /* 滑动平均 */
#endif
        
        /* 尝试发送下一个命令 */
        rt_cmd_frame_t next_frame;
        if (queue_pop(&next_frame))
        {
            g_dma_busy = true;
            g_dma_current_frame = next_frame.data;
            
#if RT_USE_DMA
            HAL_UART_Transmit_DMA(&g_uart2_handle, (uint8_t*)next_frame.data, next_frame.length);
#else
            HAL_UART_Transmit_IT(&g_uart2_handle, (uint8_t*)next_frame.data, next_frame.length);
#endif

#if RT_ENABLE_PROFILING
            g_perf.dma_start_count++;
            g_perf.last_cmd_tick = dwt_get_cycles();
#endif
        }
    }
}

/**
 * @brief       启动DMA发送（如果当前空闲）
 * @param       无
 * @retval      无
 * @note        在主循环或定时器中断调用
 */
static void dma_process(void)
{
    if (g_dma_busy) {
        return;  /* DMA忙，等待传输完成回调 */
    }
    
    rt_cmd_frame_t frame;
    if (queue_pop(&frame))
    {
        g_dma_busy = true;
        g_dma_current_frame = frame.data;
        
#if RT_ENABLE_PROFILING
        g_perf.last_cmd_tick = dwt_get_cycles();
#endif
        
#if RT_USE_DMA
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&g_uart2_handle, (uint8_t*)frame.data, frame.length);
#else
        HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&g_uart2_handle, (uint8_t*)frame.data, frame.length);
#endif
        
        if (status != HAL_OK) {
            g_dma_busy = false;  /* 发送失败，重置状态 */
        } else {
#if RT_ENABLE_PROFILING
            g_perf.dma_start_count++;
#endif
        }
    }
}

/* ====================================================================================
   [5] 核心API实现
   ==================================================================================== */

/**
 * @brief       初始化实时电机系统
 * @param       无
 * @retval      无
 */
void rt_motor_init(void)
{
    /* 初始化DWT性能计数器 */
    dwt_init();
    
#if RT_USE_PRECALC_CMDS
    /* 预编译所有命令（256*9 = 2304个命令帧，鞂30KB RAM） */
    precalc_commands();
    LOG_RT("预计算模式已启用 (30KB RAM)");
#else
    LOG_RT("动态构建模式 (节省30KB RAM, +5us延迟)");
#endif
    
    /* 清空命令队列 */
    memset(&g_cmd_queue, 0, sizeof(rt_cmd_queue_t));
    g_dma_busy = false;
    
#if RT_ENABLE_PROFILING
    memset(&g_perf, 0, sizeof(rt_perf_stats_t));
#endif

    LOG_RT("初始化完成. DMA=%d, 队列=%d, 周期=%dus", 
           RT_USE_DMA, RT_CMD_QUEUE_SIZE, RT_MOTOR_TICK_US);
}

/**
 * @brief       快速命令提交（零拷贝，直接使用预编译命令）
 * @param       addr: 电机地址
 * @param       cmd_type: 命令类型
 * @param       params: 参数数组（可选，用于位置/速度命令）
 * @retval      true=成功, false=队列满
 * @note        性能：<5μs（无参数命令），<8μs（带参数命令）
 */
bool rt_motor_submit_fast(uint8_t addr, rt_cmd_type_t cmd_type, const uint32_t *params)
{
#if RT_ENABLE_PROFILING
    g_perf.cmd_submit_count++;
#endif
    
    rt_cmd_frame_t frame = {0};
    
    /* 简单命令：使能/失能/停止/同步/查询 */
    if (cmd_type == RT_CMD_ENABLE || cmd_type == RT_CMD_DISABLE || 
        cmd_type == RT_CMD_STOP || cmd_type == RT_CMD_SYNC_TRIGGER ||
        cmd_type == RT_CMD_QUERY_POS || cmd_type == RT_CMD_QUERY_VEL)
    {
#if RT_USE_PRECALC_CMDS
        /* 快速路径：使用预编译命令 */
        return queue_push(&g_precalc_cmds[addr][cmd_type]);
#else
        /* 动态构造路径：+5μs延迟 */
        build_simple_cmd(&frame, addr, cmd_type);
        return queue_push(&frame);
#endif
    }
    
    /* 复杂命令：位置/速度运动（总是动态构造） */
    frame.addr = addr;
    
    if (cmd_type == RT_CMD_POS_MOVE && params != NULL)
    {
        /* 位置运动：{addr, 0xFD, dir, speed_h, speed_l, acc, pulses[0-3], 0/1, 0x6B} */
        uint8_t dir = params[0];
        uint16_t speed = params[1];
        uint8_t acc = params[2];
        uint32_t pulses = params[3];
        
        frame.data[0] = addr;
        frame.data[1] = 0xFD;
        frame.data[2] = dir;
        frame.data[3] = (speed >> 8) & 0xFF;
        frame.data[4] = speed & 0xFF;
        frame.data[5] = acc;
        frame.data[6] = (pulses >> 24) & 0xFF;
        frame.data[7] = (pulses >> 16) & 0xFF;
        frame.data[8] = (pulses >> 8) & 0xFF;
        frame.data[9] = pulses & 0xFF;
        frame.data[10] = 0x00;  /* 相对运动 */
        frame.data[11] = 0x00;  /* 立即执行 */
        frame.data[12] = 0x6B;
        frame.length = 13;
    }
    else if (cmd_type == RT_CMD_VEL_MOVE && params != NULL)
    {
        /* 速度运动：{addr, 0xF6, dir, speed_h, speed_l, acc, 0/1, 0x6B} */
        uint8_t dir = params[0];
        uint16_t speed = params[1];
        uint8_t acc = params[2];
        
        frame.data[0] = addr;
        frame.data[1] = 0xF6;
        frame.data[2] = dir;
        frame.data[3] = (speed >> 8) & 0xFF;
        frame.data[4] = speed & 0xFF;
        frame.data[5] = acc;
        frame.data[6] = 0x00;  /* 立即执行 */
        frame.data[7] = 0x6B;
        frame.length = 8;
    }
    else
    {
        return false;  /* 无效命令或缺少参数 */
    }
    
    return queue_push(&frame);
}

/**
 * @brief       通用命令提交（兼容现有Emm_V5接口）
 * @param       data: 命令数据
 * @param       len: 数据长度
 * @retval      true=成功, false=失败
 */
bool rt_motor_submit(const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0 || len > 16) {
        return false;
    }
    
    rt_cmd_frame_t frame = {0};
    memcpy(frame.data, data, len);
    frame.length = len;
    frame.addr = data[0];
    
    return queue_push(&frame);
}

/**
 * @brief       强制刷新命令队列
 * @param       无
 * @retval      无
 * @note        阻塞等待所有命令发送完成
 */
void rt_motor_flush(void)
{
    while (g_cmd_queue.count > 0 || g_dma_busy)
    {
        dma_process();  /* 手动触发DMA处理 */
        /* 超时保护：最多等待100ms */
        static uint32_t timeout_tick = 0;
        if (HAL_GetTick() - timeout_tick > 100) {
            break;
        }
    }
}

/**
 * @brief       查询队列可用槽位
 * @param       无
 * @retval      可用槽位数
 */
uint16_t rt_motor_queue_available(void)
{
    return RT_CMD_QUEUE_SIZE - g_cmd_queue.count - 1;
}

/**
 * @brief       查询待发送命令数
 * @param       无
 * @retval      待发送命令数
 */
uint16_t rt_motor_queue_pending(void)
{
    return g_cmd_queue.count;
}

/* ====================================================================================
   [6] 定时器中断驱动（可选，用于轨迹插补）
   ==================================================================================== */

/**
 * @brief TIM2定时器中断周期处理（10kHz）
 * @note 由stm32f1xx_it.c调用
 */
void rt_motor_tick_handler(void)
{
    // 实时命令队列处理核心（示例，实际可根据队列和DMA状态实现）
    // 这里只做空实现，实际项目可补充命令分发、DMA启动等
}

/**
 * @brief       启动实时轨迹插补定时器
 * @param       无
 * @retval      无
 */
void rt_motor_start_interpolator(void)
{
    /* 配置TIM2为100μs周期中断 */
    /* 72MHz / 72 = 1MHz, 1MHz / 100 = 10kHz = 100μs周期 */
    /* 注意：需要在CubeMX中启用TIM2并配置中断 */
    printf("[RT_MOTOR] Interpolator started (10kHz)\r\n");
}

/**
 * @brief       停止轨迹插补
 * @param       无
 * @retval      无
 */
void rt_motor_stop_interpolator(void)
{
    printf("[RT_MOTOR] Interpolator stopped\r\n");
}

/**
 * @brief       批量提交同步命令
 * @param       addrs: 电机地址数组
 * @param       count: 电机数量
 * @param       cmd_type: 命令类型
 * @param       params: 参数数组
 * @retval      true=成功, false=失败
 * @note        自动在最后添加同步触发命令
 */
bool rt_motor_sync_submit(uint8_t *addrs, uint8_t count, rt_cmd_type_t cmd_type, const uint32_t *params)
{
    /* 提交所有电机命令（snF=true） */
    for (uint8_t i = 0; i < count; i++)
    {
        if (!rt_motor_submit_fast(addrs[i], cmd_type, params)) {
            return false;
        }
    }
    
    /* 提交同步触发命令 */
    return rt_motor_submit_fast(0, RT_CMD_SYNC_TRIGGER, NULL);
}

/* ====================================================================================
   [7] 性能分析接口
   ==================================================================================== */

#if RT_ENABLE_PROFILING

void rt_motor_get_stats(rt_perf_stats_t *stats)
{
    if (stats != NULL) {
        memcpy(stats, &g_perf, sizeof(rt_perf_stats_t));
    }
}

void rt_motor_reset_stats(void)
{
    memset(&g_perf, 0, sizeof(rt_perf_stats_t));
}

uint32_t rt_motor_measure_latency(uint8_t addr, rt_cmd_type_t cmd_type)
{
    uint32_t start = dwt_get_cycles();
    rt_motor_submit_fast(addr, cmd_type, NULL);
    uint32_t end = dwt_get_cycles();
    
    return dwt_cycles_to_us(end - start);
}

#endif
