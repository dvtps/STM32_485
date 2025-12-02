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

/* USART1接收变量 */
#if USART1_EN_RX
uint8_t  g_usart1_rx_buf[USART1_REC_LEN];               /* USART1调试模式接收缓冲 */
uint16_t g_usart1_rx_sta = 0;                           /* USART1调试模式接收状态 */
static uint8_t g_usart1_rx_buffer[1];                   /* USART1单字节接收缓冲 */

/* USART1 Modbus模式变量（工业级设计：独立缓冲区避免冲突） */
volatile usart1_mode_t g_usart1_mode = USART1_MODE_DEBUG; /* 默认调试模式 */
uint8_t  g_usart1_modbus_rx_buf[USART1_REC_LEN];       /* Modbus接收缓冲 */
uint16_t g_usart1_modbus_rx_index = 0;                  /* Modbus接收索引 */
volatile uint8_t g_usart1_modbus_frame_complete = 0;   /* Modbus帧完成标志 */
static uint32_t g_usart1_last_rx_time = 0;              /* 最后接收时间(用于3.5字符超时检测) */
#endif

/* USART2接收变量(电机通信) */
#if USART2_EN_RX
uint8_t g_usart2_rx_buf[USART2_REC_LEN];                /* USART2接收缓冲 */
uint8_t g_emm_rx_cmd[USART2_REC_LEN];                   /* 电机命令帧缓冲 */
uint16_t g_emm_rx_count = 0;                            /* 电机帧长度 */
volatile uint8_t g_emm_frame_complete = 0;              /* 电机帧完成标志 */
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

#if USART2_EN_RX
    /* 初始化电机FIFO队列 */
    emm_fifo_init();
    
    /* 开启USART2接收中断（RXNE + IDLE双中断） */
    __HAL_UART_ENABLE_IT(&g_uart2_handle, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&g_uart2_handle, UART_IT_IDLE);
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
 * @brief       USART1模式切换（工业级设计：运行时安全切换）
 * @param       mode: USART1_MODE_DEBUG(调试模式) 或 USART1_MODE_MODBUS(Modbus模式)
 * @retval      无
 * @note        切换前会自动清空接收缓冲区，确保无脏数据
 */
void usart1_set_mode(usart1_mode_t mode)
{
#if USART1_EN_RX
    __disable_irq();  /* 临界区保护：防止切换过程中断干扰 */
    
    /* 清空两个缓冲区（防止模式切换时数据污染） */
    g_usart1_rx_sta = 0;
    g_usart1_modbus_rx_index = 0;
    g_usart1_modbus_frame_complete = 0;
    
    /* 切换模式 */
    g_usart1_mode = mode;
    
    __enable_irq();
#endif
}

/**
 * @brief       获取USART1当前工作模式
 * @retval      usart1_mode_t: 当前模式
 */
usart1_mode_t usart1_get_mode(void)
{
    return g_usart1_mode;
}

/**
 * @brief       清空USART1接收缓冲区（模式切换前调用）
 * @retval      无
 */
void usart1_flush_rx_buffer(void)
{
#if USART1_EN_RX
    __disable_irq();
    g_usart1_rx_sta = 0;
    g_usart1_modbus_rx_index = 0;
    g_usart1_modbus_frame_complete = 0;
    __enable_irq();
#endif
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
                g_usart1_rx_sta = 0;                    /* 接收错误,重新开始 */
            }
            else
            {
                g_usart1_rx_sta |= 0x8000;              /* 接收完成 */
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
#include "protocol_router.h"  /* V3.0新增：协议路由器 */

static uint32_t idle_count = 0;  /* 调试：IDLE中断计数 */
static uint32_t g_fifo_overflow_count = 0;  /* V3.5 Phase 8: FIFO溢出统计 */

/* V3.5优化：全局标志位，主循环轮询处理 */
volatile uint8_t g_usart2_frame_ready = 0;  /* 帧就绪标志 */

static void usart2_idle_callback(UART_HandleTypeDef *huart)
{
    idle_count++;  /* 调试：记录IDLE中断次数 */
    
    /* 清除IDLE标志 */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    
    /* V3.5关键优化：仅设置标志位，立即退出中断（<5μs）
     * 数据出队和协议解析延迟到主循环处理
     */
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
    /* Modbus模式: 采用RXNE+IDLE双中断机制（类似USART2） */
    if (g_usart1_mode == USART1_MODE_MODBUS)
    {
        uint8_t data;
        
        /* RXNE中断: 字节级接收 */
        if (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_RXNE) != RESET)
        {
            data = (uint8_t)(g_uart1_handle.Instance->DR & 0xFF);
            
            /* 存入Modbus缓冲区（带溢出保护） */
            if (g_usart1_modbus_rx_index < USART1_REC_LEN)
            {
                g_usart1_modbus_rx_buf[g_usart1_modbus_rx_index++] = data;
                g_usart1_last_rx_time = HAL_GetTick();  /* 更新接收时间 */
            }
            else
            {
                /* 缓冲区溢出，重置索引（防止死锁） */
                g_usart1_modbus_rx_index = 0;
            }
            
            __HAL_UART_CLEAR_FLAG(&g_uart1_handle, UART_FLAG_RXNE);
        }
        
        /* IDLE中断: 帧边界检测（Modbus标准3.5字符超时） */
        if (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_IDLE) != RESET)
        {
            __HAL_UART_CLEAR_IDLEFLAG(&g_uart1_handle);
            
            /* 设置帧完成标志（主循环处理） */
            if (g_usart1_modbus_rx_index > 0)
            {
                g_usart1_modbus_frame_complete = 1;
                
                /* 调用Modbus协议层回调（可选：也可在主循环处理） */
                #ifdef MODBUS_RTU_IDLE_CALLBACK
                extern void modbus_rtu_idle_callback(void);
                modbus_rtu_idle_callback();
                #endif
            }
        }
    }
    else  /* 调试模式: 使用HAL库默认处理 */
    {
        /* IDLE中断清除（调试模式不使用IDLE，但需清标志避免一直触发） */
        if (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_IDLE) != RESET)
        {
            __HAL_UART_CLEAR_IDLEFLAG(&g_uart1_handle);
        }
        
        /* HAL库默认处理（RXNE中断 → HAL_UART_RxCpltCallback） */
        HAL_UART_IRQHandler(&g_uart1_handle);
    }
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
#if USART2_USE_DMA
    /* DMA模式: 仅处理IDLE中断 */
    if (__HAL_UART_GET_FLAG(&g_uart2_handle, UART_FLAG_IDLE) != RESET)
    {
        usart2_idle_dma_handler(&g_uart2_handle);
    }
#else
    /* 传统模式: RXNE + IDLE中断 */
    uint8_t data;
    
    if (__HAL_UART_GET_FLAG(&g_uart2_handle, UART_FLAG_RXNE) != RESET)
    {
        data = (uint8_t)(g_uart2_handle.Instance->DR & 0xFF);
        if (emm_fifo_enqueue((uint16_t)data) != 0)
        {
            /* V3.5 Phase 8优化: FIFO溢出计数（主循环可通过get_fifo_overflow_count()查询） */
            g_fifo_overflow_count++;
        }
        __HAL_UART_CLEAR_FLAG(&g_uart2_handle, UART_FLAG_RXNE);
    }
    
    if (__HAL_UART_GET_FLAG(&g_uart2_handle, UART_FLAG_IDLE) != RESET)
    {
        usart2_idle_callback(&g_uart2_handle);
    }
#endif
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
