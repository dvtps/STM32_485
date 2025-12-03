# 实时模式启用指南

## ⚠️ 重要提示

当前系统**缺少DMA配置**，需要通过STM32CubeMX重新配置才能使用实时模式。

---

## 🔧 方法1：通过CubeMX配置DMA（推荐）

### 步骤1：打开工程配置文件
```
双击打开: STM32_485.ioc
```

### 步骤2：配置USART2的DMA
```
1. 左侧菜单 → Connectivity → USART2
2. 点击 "DMA Settings" 标签页
3. 点击 "Add" 按钮
4. 配置如下：
   - DMA Request: USART2_TX
   - Channel: DMA1 Channel 7
   - Direction: Memory To Peripheral
   - Priority: Very High
   - Mode: Normal
   - Increment Address: Memory (勾选)
   - Data Width: Byte (保持默认)
```

### 步骤3：配置DMA中断优先级
```
1. 左侧菜单 → System Core → NVIC
2. 找到 "DMA1 channel7 global interrupt"
3. 勾选 "Enabled"
4. 设置优先级：
   - Preemption Priority: 0 (最高)
   - Sub Priority: 0
```

### 步骤4：生成代码
```
1. 点击顶部 "Project" → "Generate Code"
2. 确认生成完成
```

### 步骤5：重新编译
```powershell
cmake --build --preset Debug
```

---

## 🚀 方法2：手动添加DMA配置代码（快速但需谨慎）

如果不想使用CubeMX，可以手动添加以下代码：

### 修改1：usart.c 添加DMA句柄

在 `Drivers/SYSTEM/usart/usart.c` 顶部添加：

```c
/* DMA句柄（仅实时模式使用） */
#if REALTIME_MOTOR_ENABLE
DMA_HandleTypeDef hdma_usart2_tx;
#endif
```

### 修改2：usart.c 初始化函数中添加DMA配置

在 `usart_init()` 函数的USART2初始化后添加：

```c
#if REALTIME_MOTOR_ENABLE
    /* 使能DMA1时钟 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    /* 配置USART2_TX的DMA */
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
        Error_Handler();
    }
    
    /* 关联DMA到UART */
    __HAL_LINKDMA(&g_uart2_handle, hdmatx, hdma_usart2_tx);
    
    /* 使能DMA中断 */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
#endif
```

### 修改3：stm32f1xx_it.c 添加DMA中断处理

在 `Core/Src/stm32f1xx_it.c` 中添加：

```c
/* 在文件顶部添加extern声明 */
#if REALTIME_MOTOR_ENABLE
extern DMA_HandleTypeDef hdma_usart2_tx;
#endif

/* 在中断函数区域添加 */
#if REALTIME_MOTOR_ENABLE
/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}
#endif
```

---

## ✅ 验证DMA配置

编译后运行以下测试代码：

```c
/* 在main.c的USER CODE BEGIN 2添加 */
#if REALTIME_MOTOR_ENABLE
    printf("DMA State: %d\r\n", g_uart2_handle.hdmatx->State);
    printf("Expected: 1 (HAL_DMA_STATE_READY)\r\n");
#endif
```

如果输出 `DMA State: 1`，说明DMA配置成功。

---

## 📝 已自动修改的文件

我已经为您修改了以下文件以启用实时模式：

### ✅ 1. app_config.h
```c
#define REALTIME_MOTOR_ENABLE       1  /* 已启用 */
```

### ✅ 2. main.c
已添加：
```c
#include "realtime_motor.h"

/* 在初始化区域 */
#if REALTIME_MOTOR_ENABLE
    rt_motor_init();
    printf("Real-Time Motor System Initialized\r\n");
#endif
```

---

## ⚠️ 下一步操作

**请选择一种方法完成DMA配置：**

1. **推荐**：使用CubeMX配置（方法1）- 最安全
2. **快速**：手动添加代码（方法2）- 需要仔细

完成DMA配置后：
```powershell
cmake --build --preset Debug
```

---

## 📊 预期效果

启用实时模式后，您将看到：

```
[RT_MOTOR] Initialized. DMA=1, Queue=32, Tick=100us
Real-Time Motor System Initialized
```

然后可以使用微秒级API：
```c
RT_MOTOR_ENABLE(0x01);              /* <5μs */
RT_MOTOR_POS(0x01, 0, 800, 20, 3200); /* <8μs */
```

---

**当前状态**: app_config.h已设置`REALTIME_MOTOR_ENABLE=1`，但需要配置DMA才能编译通过。
