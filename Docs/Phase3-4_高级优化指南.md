# Phase 3-4 高级优化使用指南

## ✅ 已实施模块

### 1. DMA接收模式（Phase 3-1）

**文件**: `Drivers/SYSTEM/usart/usart_dma.c/h`

**启用方法**:
```c
// 在 usart.c 第14行修改:
#define USART2_USE_DMA  1  // 改为1启用DMA
```

**工作原理**:
- DMA1 Channel6循环接收到512字节缓冲区
- IDLE中断触发时计算接收长度并批量入FIFO
- CPU占用从约5%降至<0.5%（115200bps时）

**适用场景**:
- 多轴电机控制（3轴以上）
- 高速通信（波特率>115200）
- CPU资源紧张时

**验证方法**:
```c
// 在main.c中添加:
extern uint8_t usart2_dma_get_usage(void);
uint8_t dma_usage = usart2_dma_get_usage();
printf("DMA缓冲区使用率: %d%%\r\n", dma_usage);
```

---

### 2. TIM3定时看门狗（Phase 4-3）

**文件**: `Drivers/BSP/IWDG/iwdg_timer.c/h`

**启用方法**:
```c
// 在 main.c 第87行附近修改:
#if 1  // 改为1启用TIM3定时喂狗
extern void tim3_watchdog_init(void);
tim3_watchdog_init();
#endif
```

**同时修改主循环**:
```c
// 在 main.c while(1)中修改:
#if 1  // 与上面保持一致
extern void main_loop_heartbeat_update(void);
main_loop_heartbeat_update();  // 更新心跳
#else
iwdg_feed();  // 传统模式
#endif
```

**健康检查项**:
1. FIFO水位 < 90%
2. 主循环心跳500ms内更新
3. 电机UART错误增长 < 10次/500ms

**失败行为**:
- 停止喂狗 → 2秒后看门狗复位
- 可在复位前记录故障状态到FLASH

**调试接口**:
```c
extern uint32_t get_health_check_fail_count(void);
uint32_t fail_cnt = get_health_check_fail_count();
```

---

## 🔧 性能对比

| 模式 | CPU占用 | 中断频率 | 内存占用 | 适用场景 |
|------|---------|----------|----------|----------|
| RXNE中断 | ~5% | 200+次/秒 | +512B FIFO | 单轴/双轴 |
| DMA循环 | <0.5% | 仅IDLE | +512B DMA + 512B FIFO | 多轴/高速 |

| 看门狗模式 | 喂狗频率 | 故障检测 | 代码复杂度 |
|-----------|----------|----------|-----------|
| 主循环 | ~1000Hz | 无 | 低 |
| TIM3定时 | 2Hz | 3项健康检查 | 中等 |

---

## ⚠️ 注意事项

### DMA模式

1. **内存对齐**: DMA缓冲区已声明为4字节对齐
2. **缓存一致性**: STM32F103无DCache，无需考虑
3. **FIFO深度**: 当前512字节足够，如需更大可同步修改
4. **调试困难**: DMA模式下无法在RXNE中断打断点

### TIM3看门狗

1. **健康阈值**: 根据实际应用调整（当前90% FIFO水位）
2. **主循环阻塞**: 任何>500ms的阻塞都会触发复位
3. **中断优先级**: TIM3=2，必须低于USART2(1)
4. **故障记录**: 建议在复位前写备份寄存器保存状态

---

## 📊 资源占用（全启用）

```
Flash: 26012 B (39.69%) - 增加880B
RAM:   4296 B (20.98%) - 增加152B（TIM3状态变量）

增量主要来自:
- usart_dma.c: ~600B Flash
- iwdg_timer.c: ~280B Flash
- DMA缓冲区: 512B RAM（已包含在4296中）
```

---

## 🚀 推荐配置

### 开发阶段（当前默认）
```c
#define USART2_USE_DMA  0  // 关闭，便于调试
// TIM3看门狗: 关闭
```

### 生产环境（单轴/双轴）
```c
#define USART2_USE_DMA  0  // 关闭，RXNE足够
// TIM3看门狗: 启用，增强可靠性
```

### 生产环境（多轴/高速）
```c
#define USART2_USE_DMA  1  // 启用，降低CPU占用
// TIM3看门狗: 启用，全面保护
```

---

## 🔍 故障排查

**DMA模式无数据**:
1. 检查`DMA1_Channel6_IRQHandler`是否在startup文件中
2. 确认`HAL_UART_Receive_DMA()`返回HAL_OK
3. 使用逻辑分析仪确认USART2有数据输入

**TIM3看门狗误复位**:
1. 增加主循环心跳频率（当前500ms阈值）
2. 临时关闭健康检查项定位问题
3. 查看`get_health_check_fail_count()`确定失败原因

**内存不足**:
- DMA缓冲区占用512B，如RAM紧张可改为256B
- 同步修改`usart_dma.c`中的缓冲区和计数器大小

---

*本文档基于 2025-12-01 优化版本，与代码同步维护*
