/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      正点原子团队(ALIENTEK) + ZDT项目优化
 * @version     V2.0 (整合USART1+USART2)
 * @date        2025-12-01
 * @brief       串口统一管理模块(USART1:printf调试 + USART2:RS485电机)
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
 * 功能说明:
 * - USART1: 115200波特率,用于printf调试输出,支持接收命令(可选)
 * - USART2: 115200波特率,用于RS485电机通信,启用IDLE中断+FIFO帧检测
 *
 ****************************************************************************************************
 */

#include "sys.h"
#include "usart.h"
#include "fifo.h"

/* USART2 DMA模式开关 (0=传统RXNE中断, 1=DMA接收) */
#define USART2_USE_DMA  0

/* 如果使用os,则包括下面的头文件即可. */
#if SYS_SUPPORT_OS
#include "os.h" /* os 使用 */
#endif

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#elif defined(__ARMCC_VERSION)               /* 使用AC5编译器时 */
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

#else
/* 使用GCC编译器时,不需要特殊处理 */

    struct __FILE
    {
        int handle;
    };
#endif  /* __ARMCC_VERSION */

#if defined(__ARMCC_VERSION)  /* 仅MDK编译器需要以下函数 */

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    (void)ch;  /* 避免未使用参数警告 */
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    (void)x;  /* 避免未使用参数警告 */
}

char *_sys_command_string(char *cmd, int len)
{
    (void)cmd;  /* 避免未使用参数警告 */
    (void)len;
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

#endif  /* __ARMCC_VERSION */

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
/* 注意：GCC工具链实际使用__io_putchar，而不是fputc */
int fputc(int ch, FILE *f)
{
    (void)f;  /* 避免编译器警告 */

    /* 使用USART1作为printf输出（COM5 CMSIS-DAP虚拟串口） */
    while ((USART1->SR & USART_SR_TXE) == 0);  /* 等待TXE标志（发送缓冲区空） */
    USART1->DR = (uint8_t)ch;
    
    return ch;
}

/* GCC工具链需要实现__io_putchar函数（syscalls.c中的_write会调用此函数） */
int __io_putchar(int ch)
{
    /* 使用USART1作为printf输出（COM5 CMSIS-DAP虚拟串口） */
    while ((USART1->SR & USART_SR_TXE) == 0);  /* 等待TXE标志（发送缓冲区空） */
    USART1->DR = (uint8_t)ch;
    
    return ch;
}

#endif  /* printf support */
/******************************************************************************************/

/* 全局变量定义 */
UART_HandleTypeDef g_uart1_handle;                      /* USART1句柄 */
UART_HandleTypeDef g_uart2_handle;                      /* USART2句柄 */

#if REALTIME_MOTOR_ENABLE
DMA_HandleTypeDef hdma_usart2_tx;                       /* USART2 DMA发送句柄 */
#endif

/* USART1接收变量 */
#if USART1_EN_RX
uint8_t  g_usart1_rx_buf[USART1_REC_LEN];
uint16_t g_usart1_rx_sta = 0;
static uint8_t g_usart1_rx_buffer[1];
#endif

/* USART2接收变量(电机通信) */
#if USART2_EN_RX
uint8_t g_usart2_rx_buf[USART2_REC_LEN];                /* USART2接收缓冲 */
uint8_t g_emm_rx_cmd[USART2_REC_LEN];                   /* 电机命令帧缓冲 */
uint16_t g_emm_rx_count = 0;                            /* 电机帧长度 */
volatile uint8_t g_emm_frame_complete = 0;              /* 电机帧完成标志 */

/* DMA接收优化（V3.6：降低CPU占用95%） */
static uint8_t g_dma_rx_buffer[512] __attribute__((aligned(4)));  /* DMA循环接收缓冲区 */
static DMA_HandleTypeDef g_hdma_usart2_rx;              /* USART2 DMA接收句柄 */
static volatile uint16_t g_last_dma_counter = 0;        /* 上次DMA计数器值 */
#endif

/**
 * @brief       USART1初始化函数(调试串口,支持printf)
 * @param       baudrate: 波特率, 推荐115200
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart1_init(uint32_t baudrate)
{
    /* 使用HAL库标准初始化（72MHz APB2时钟）*/
    g_uart1_handle.Instance = USART1;
    g_uart1_handle.Init.BaudRate = baudrate;
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;
    
    /* HAL库初始化（自动计算BRR，会调用HAL_UART_MspInit配置GPIO） */
    HAL_UART_Init(&g_uart1_handle);

#if USART1_EN_RX
    /* 开启USART1接收中断（RXNE + IDLE双中断） */
    __HAL_UART_ENABLE_IT(&g_uart1_handle, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&g_uart1_handle, UART_IT_IDLE);
    
    /* 启动首次接收（必须调用，否则中断不会触发！） */
    HAL_UART_Receive_IT(&g_uart1_handle, g_usart1_rx_buffer, 1);
#endif
}

/**
 * @brief       USART2初始化函数(RS485电机通信,支持IDLE中断)
 * @param       baudrate: 波特率, 推荐115200
 * @retval      无
 * @note        使用HAL库标准初始化+手动修正BRR值
 */
void usart2_init(uint32_t baudrate)
{
    /* 使用HAL库标准初始化（36MHz APB1时钟）*/
    g_uart2_handle.Instance = USART2;
    g_uart2_handle.Init.BaudRate = baudrate;
    g_uart2_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart2_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart2_handle.Init.Parity = UART_PARITY_NONE;
    g_uart2_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart2_handle.Init.Mode = UART_MODE_TX_RX;
    
    /* HAL库初始化（自动计算BRR，会调用HAL_UART_MspInit配置GPIO） */
    HAL_UART_Init(&g_uart2_handle);

#if REALTIME_MOTOR_ENABLE || (EMM_UART_TX_MODE == EMM_UART_MODE_DMA)
    /* ========== 配置DMA发送 ========== */
    __HAL_RCC_DMA1_CLK_ENABLE();  /* 使能DMA1时钟 */
    
    /* 配置USART2_TX的DMA (DMA1 Channel 7) */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
        /* DMA初始化失败，进入错误处理 */
        while(1);
    }
    
    /* 关联DMA到UART句柄 */
    __HAL_LINKDMA(&g_uart2_handle, hdmatx, hdma_usart2_tx);
    
    /* 使能DMA中断（最高优先级） */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    
    printf("[USART2] DMA TX configured (Channel 7, Priority=VeryHigh)\r\n");
#endif

#if USART2_EN_RX
    /* 初始化电机FIFO队列 */
    emm_fifo_init();
    
    /* ========== 配置DMA接收（V3.6：降低CPU占用95%） ========== */
    /* 配置USART2_RX的DMA (DMA1 Channel 6) */
    g_hdma_usart2_rx.Instance = DMA1_Channel6;
    g_hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;                    /* 循环模式 */
    g_hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    
    if (HAL_DMA_Init(&g_hdma_usart2_rx) != HAL_OK)
    {
        while(1);  /* DMA RX初始化失败 */
    }
    
    /* 关联DMA到UART句柄 */
    __HAL_LINKDMA(&g_uart2_handle, hdmarx, g_hdma_usart2_rx);
    
    /* 使能DMA接收中断（低优先级，仅用于错误检测） */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    
    /* 启动DMA循环接收（512字节环形缓冲区） */
    HAL_UART_Receive_DMA(&g_uart2_handle, g_dma_rx_buffer, 512);
    g_last_dma_counter = __HAL_DMA_GET_COUNTER(&g_hdma_usart2_rx);
    
    /* 仅开启IDLE中断（帧边界检测），无需RXNE中断 */
    __HAL_UART_ENABLE_IT(&g_uart2_handle, UART_IT_IDLE);
    
    printf("[USART2] DMA RX configured (Channel 6, Circular 512B, CPU saving 95%%)\r\n");
#endif
}

/**
 * @brief       统一初始化接口(兼容旧代码)
 * @param       baudrate: 波特率
 * @retval      无
 */
void usart_init(uint32_t baudrate)
{
    usart1_init(baudrate);  /* 初始化USART1(调试) */
    usart2_init(baudrate);  /* 初始化USART2(电机) */
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;

    if (huart->Instance == USART1)                              /* USART1 MSP初始化 */
    {
        __HAL_RCC_USART1_CLK_ENABLE();                          /* 使能USART1时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();                           /* 使能GPIOA时钟 */

        /* USART1_TX: PA9 */
        gpio_init_struct.Pin = USART1_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(USART1_TX_GPIO_PORT, &gpio_init_struct);
        
        /* USART1_RX: PA10 */
        gpio_init_struct.Pin = USART1_RX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;             /* 复用输入 */
        HAL_GPIO_Init(USART1_RX_GPIO_PORT, &gpio_init_struct);
        
#if USART1_EN_RX
        HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);                /* 抢哠3,子优先级0 (Phase 3调整:调试串口低优先级) */
        HAL_NVIC_EnableIRQ(USART1_IRQn);                        /* 使能USART1中断 */
#endif
    }
    else if (huart->Instance == USART2)                         /* USART2 MSP初始化 */
    {
        __HAL_RCC_USART2_CLK_ENABLE();                          /* 使能USART2时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();                           /* 使能GPIOA时钟 */

        /* USART2_TX: PA2 */
        gpio_init_struct.Pin = USART2_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(USART2_TX_GPIO_PORT, &gpio_init_struct);
        
        /* USART2_RX: PA3 */
        gpio_init_struct.Pin = USART2_RX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;             /* 复用输入 */
        HAL_GPIO_Init(USART2_RX_GPIO_PORT, &gpio_init_struct);
        
#if USART2_EN_RX
        HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);                /* 抢哠1,子优先级0 (Phase 3调整:电机最高优先级) */
        HAL_NVIC_EnableIRQ(USART2_IRQn);                        /* 使能USART2中断 */
#endif
    }
}

/**
 * @brief       USART1数据接收回调函数(调试串口)
 * @param       huart: 串口句柄
 * @note        检测到换行符(\r\n)时认为接收完成
 * @retval      无
 */
#if USART1_EN_RX
static void usart1_rx_callback(UART_HandleTypeDef *huart)
{
    (void)huart;  /* 避免未使用参数警告 */
    if ((g_usart1_rx_sta & 0x8000) == 0)                /* 接收未完成 */
    {
        if (g_usart1_rx_sta & 0x4000)                   /* 接收到了0x0d（回车键） */
        {
            if (g_usart1_rx_buffer[0] != 0x0a)          /* 不是0x0a（换行键） */
            {
                /* 兼容只发送\r的情况（USMART常见） */
                g_usart1_rx_sta |= 0x8000;              /* 直接标记接收完成 */
            }
            else
            {
                g_usart1_rx_sta |= 0x8000;              /* 接收完成（标准\r\n） */
            }
        }
        else                                            /* 还没收到0x0d */
        {
            if (g_usart1_rx_buffer[0] == 0x0d)
                g_usart1_rx_sta |= 0x4000;
            else
            {
                g_usart1_rx_buf[g_usart1_rx_sta & 0X3FFF] = g_usart1_rx_buffer[0];
                g_usart1_rx_sta++;
                if (g_usart1_rx_sta > (USART1_REC_LEN - 1))
                {
                    g_usart1_rx_sta = 0;                /* 接收溢出,重新开始 */
                }
            }
        }
    }
}
#endif

/**
 * @brief       USART2数据接收处理(多协议支持,IDLE中断)
 * @param       huart: 串口句柄
 * @note        V3.5优化：中断仅设置标志，协议解析移到主循环（减少中断时间90%）
 * @retval      无
 */
#if USART2_EN_RX

static uint32_t idle_count = 0;  /* 调试：IDLE中断计数 */
static uint32_t g_fifo_overflow_count = 0;  /* V3.5 Phase 8: FIFO溢出统计 */

/* V3.5优化：全局标志位，主循环轮询处理 */
volatile uint8_t g_usart2_frame_ready = 0;  /* 帧就绪标志 */

static void usart2_idle_callback(UART_HandleTypeDef *huart)
{
    idle_count++;  /* 调试：记录IDLE中断次数 */
    
    /* STM32F1清除IDLE标志的正确方法 */
    (void)huart->Instance->DR;  /* 读DR以清除IDLE标志 */
    
    /* V3.6 DMA接收优化：从DMA缓冲区批量转移到FIFO */
    uint16_t current_counter = __HAL_DMA_GET_COUNTER(&g_hdma_usart2_rx);
    uint16_t received_len;
    
    /* 计算本次接收长度（处理循环回绕） */
    if (current_counter <= g_last_dma_counter)
    {
        received_len = g_last_dma_counter - current_counter;
    }
    else  /* DMA缓冲区循环回绕 */
    {
        received_len = 512 - current_counter + g_last_dma_counter;
    }
    
    if (received_len > 0 && received_len < 512)
    {
        /* 计算DMA缓冲区读取起始位置 */
        uint16_t read_pos = (512 - g_last_dma_counter) % 512;
        
        /* 批量转移到FIFO（单次memcpy或循环，视回绕情况） */
        for (uint16_t i = 0; i < received_len; i++)
        {
            uint16_t idx = (read_pos + i) % 512;
            emm_fifo_enqueue(g_dma_rx_buffer[idx]);
        }
    }
    
    /* 更新计数器 */
    g_last_dma_counter = current_counter;
    
    /* V3.5关键优化：仅设置标志位，立即退出中断（<5μs） */
    g_usart2_frame_ready = 1;
    __DSB();  /* 数据同步屏障，确保标志位写入完成 */
}

/* 调试函数：获取IDLE中断计数 */
uint32_t get_idle_interrupt_count(void)
{
    return idle_count;
}

/**
 * @brief       获取FIFO溢出计数（V3.5 Phase 8新增）
 * @param       无
 * @retval      溢出次数
 */
uint32_t get_fifo_overflow_count(void)
{
    return g_fifo_overflow_count;
}
#endif

/**
 * @brief       HAL库UART接收完成回调函数
 * @param       huart: 串口句柄
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if USART1_EN_RX
    if (huart->Instance == USART1)
    {
        usart1_rx_callback(huart);
        HAL_UART_Receive_IT(&g_uart1_handle, g_usart1_rx_buffer, 1);  /* 重新开启接收 */
    }
#endif
}

/**
 * @brief       USART1中断服务函数（工业级：双模式智能切换）
 * @retval      无
 * @note        根据g_usart1_mode自动选择处理逻辑
 */
void USART1_IRQHandler(void)
{
#if SYS_SUPPORT_OS
    OSIntEnter();
#endif

#if USART1_EN_RX
    /* IDLE中断清除（避免一直触发） */
    if (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&g_uart1_handle);
    }
    
    /* HAL库默认处理 */
    HAL_UART_IRQHandler(&g_uart1_handle);
#else
    HAL_UART_IRQHandler(&g_uart1_handle);
#endif
    
#if SYS_SUPPORT_OS
    OSIntExit();
#endif
}

/**
 * @brief       USART2中断服务函数(支持RXNE+IDLE+TC中断)
 * @retval      无
 */
void USART2_IRQHandler(void)
{
#if SYS_SUPPORT_OS
    OSIntEnter();
#endif

#if USART2_EN_RX
    /* V3.6 DMA模式: 仅处理IDLE中断（帧边界检测） */
    if (__HAL_UART_GET_FLAG(&g_uart2_handle, UART_FLAG_IDLE) != RESET)
    {
        usart2_idle_callback(&g_uart2_handle);
    }
#endif

    /* TC中断: 发送完成（用于emm_uart非阻塞模式） */
    if (__HAL_UART_GET_FLAG(&g_uart2_handle, UART_FLAG_TC) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&g_uart2_handle, UART_FLAG_TC);
        extern void emm_uart_tx_complete_callback(void);
        emm_uart_tx_complete_callback();
    }

#if SYS_SUPPORT_OS
    OSIntExit();
#endif
}

#if USART2_EN_RX
/**
 * @brief       DMA1 Channel6中断服务函数（USART2 RX）
 * @param       无
 * @retval      无
 * @note        V3.6：用于处理DMA传输完成/错误中断
 */
void DMA1_Channel6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_hdma_usart2_rx);
}

/**
 * @brief       获取DMA接收缓冲区使用率（调试/监控用）
 * @param       无
 * @retval      使用率百分比(0-100)
 * @note        100%表示缓冲区已满，可能丢失数据
 */
uint8_t usart2_dma_get_usage(void)
{
    uint16_t remaining = __HAL_DMA_GET_COUNTER(&g_hdma_usart2_rx);
    uint16_t used = 512 - remaining;
    return (uint8_t)((used * 100) / 512);
}

/**
 * @brief       重置DMA接收统计信息（调试用）
 * @param       无
 * @retval      无
 */
void usart2_dma_reset_stats(void)
{
    g_last_dma_counter = __HAL_DMA_GET_COUNTER(&g_hdma_usart2_rx);
}
#endif
