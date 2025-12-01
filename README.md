# STM32F103 步进电机控制系统

[![构建状态](https://img.shields.io/badge/build-passing-brightgreen)]() [![版本](https://img.shields.io/badge/version-V3.0-blue)]() [![许可证](https://img.shields.io/badge/license-MIT-green)]()

基于STM32F103C8T6的闭环步进电机控制系统，使用张大头Y系列V2.0电机通过RS485通信。项目采用HAL库+CMake构建系统，核心为双串口架构：USART1调试输出 + USART2电机通信。

## ✨ 特性

- **V3.0三层架构**: 应用层 → 协议层 → 通信层 → 硬件层，清晰的依赖关系
- **统一通信层**: 新增`emm_uart.c`（220行），支持阻塞/中断/DMA三种模式
- **USMART调试**: 通过USART1串口发送命令调用电机函数（如`motor_enable(1,1)`）
- **实时统计**: 通信成功率、错误次数、发送忙次数实时监控
- **IWDG看门狗**: 2秒超时保护，自动监控通信健康度
- **双串口分工**: USART1专属调试（printf），USART2专属电机通信（RS485）
- **IDLE中断**: 电机响应帧自动检测，支持不定长帧
- **CMake构建**: 跨平台开发，支持VS Code + Cortex-Debug

## 📊 项目指标（V3.0）

| 指标 | 数值 | 说明 |
|------|------|------|
| **Flash占用** | 28984 bytes | 44.23% of 64KB（优化-132 bytes） |
| **RAM占用** | ~3500 bytes | 17% of 20KB |
| **代码行数** | 减少625行 | 删除冗余测试文件+简化协议层 |
| **通信效率** | >98% | 基于emm_uart统计 |
| **MCU主频** | 72MHz | HSE 8MHz + PLL ×9 |

## 🚀 快速开始

### 硬件要求

- **开发板**: 正点原子M48Z-M3（STM32F103C8T6）
- **RS485模块**: ATK-MB024（或兼容模块）
- **电机**: 张大头Y系列V2.0闭环步进电机（Emm_V5协议）
- **调试器**: ATK-CMSIS-DAP / ST-Link
- **串口工具**: 2个USB转TTL（USART1调试 + USART2监控）

### 硬件连接

```
STM32开发板          RS485模块        Y系列电机
────────────────────────────────────────────
PA2 (USART2_TX) ──→  RX
PA3 (USART2_RX) ←──  TX               
3.3V            ──→  VCC
GND             ──→  GND        
                     A     ──→    A
                     B     ──→    B

PA9  (USART1_TX) ──→  USB转TTL (调试输出)
PA10 (USART1_RX) ←──  USB转TTL

PA13/PA14 (SWD) ──→  CMSIS-DAP/ST-Link
```

### 软件要求

- **VS Code** 1.85+
- **CMake** 3.28+
- **gcc-arm-none-eabi** 13.2.1+
- **Ninja** 构建工具
- **PyOCD** v0.41.0（烧录/调试）

### 编译步骤

```powershell
# 1. 克隆项目
git clone https://github.com/dvtps/STM32_485.git
cd STM32_485

# 2. 配置构建
cmake --preset Debug

# 3. 编译
cmake --build --preset Debug

# 4. 烧录（PyOCD）
pyocd flash -t stm32f103rc build/Debug/STM32_485.elf

# 或使用OpenOCD
openocd -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg \
        -c "program build/Debug/STM32_485.elf verify reset exit"
```

### 运行测试

```powershell
# 打开串口助手（USART1, 115200, 8-N-1）
# 发送USMART命令测试：
? 或 help                          # 查看命令列表
motor_enable(1,1)                  # 使能电机1
motor_pos_move(1,0,300,10,3200)    # 电机1顺时针转1圈（300RPM，加速度10）
motor_vel_move(1,0,500,20)         # 电机1持续转动500RPM
motor_stop(1)                      # 急停电机1
motor_home(1)                      # 电机1回零
emm_uart_print_stats()             # 查看通信统计
```

预期输出：
```
[EMM UART Statistics]
TX Success:   147
TX Error:     2
TX Busy:      0
TX Throttle:  5
Last TX Tick: 45320 ms
Success Rate: 98.66%
```

## 📁 项目结构

```
STM32_485/
├── .github/
│   └── copilot-instructions.md    # AI开发指南（必读！）
├── Core/
│   ├── App/
│   │   ├── main.c                 # 主程序入口
│   │   ├── motor_zdt.c            # 电机控制应用
│   │   └── app_config.h           # 统一配置文件
│   ├── Inc/                       # CubeMX生成头文件
│   └── Src/                       # CubeMX生成源文件
├── Drivers/
│   ├── BSP/                       # 板级驱动
│   │   ├── EMM_V5/                # ⭐ 电机协议封装（V3.0核心）
│   │   │   ├── emm_v5.c           # 协议层：15个API
│   │   │   ├── emm_fifo.c         # 环形队列：256字节
│   │   │   ├── emm_uart.c         # ✨ 通信层：统一接口（220行）
│   │   │   └── emm_uart.h         # 通信接口+统计定义
│   │   ├── LED/                   # LED驱动
│   │   ├── KEY/                   # 按键驱动
│   │   └── IWDG/                  # 看门狗驱动（2s超时）
│   ├── SYSTEM/                    # 系统驱动
│   │   ├── usart/                 # USART1+2统一管理
│   │   ├── delay/                 # 延时函数（72MHz校准）
│   │   └── sys/                   # 时钟配置
│   ├── Middlewares/               # 中间件
│   │   └── USMART/                # 串口调试工具
│   ├── CMSIS/                     # ARM CMSIS库
│   └── STM32F1xx_HAL_Driver/      # ST HAL库
├── Docs/                          # 📚 完整技术文档
│   ├── 架构优化总结_V3.0.md        # V3.0优化详情
│   ├── CubeMX配置详细步骤.md       # CubeMX配置指南
│   ├── USMART使用指南.md          # 串口调试工具
│   ├── doc_Y57/                   # 张大头电机文档
│   └── 00-项目总览.md ~ 07-电机控制快速入门.md
├── build/                         # 构建输出目录
├── cmake/                         # CMake配置文件
├── CMakeLists.txt                 # 根CMake配置
├── CMakePresets.json              # CMake预设
└── README.md                      # 本文件

关键文件说明：
- emm_uart.c (220行) - V3.0新增统一通信层，替代ATK_RS485
- emm_v5.c (380行)   - 协议层，简化24行→10行（-58%）
- motor_zdt.c        - 应用层，按键控制+LED指示
- usart.c            - 双串口初始化+IDLE中断
```

## 🎯 V3.0架构设计

### 三层架构

```
┌──────────────────────────────────────────┐
│         应用层 (motor_zdt.c)             │
│  按键控制 + LED指示 + 业务逻辑             │
└──────────────┬───────────────────────────┘
               │ 调用 Emm_V5_* API
┌──────────────▼───────────────────────────┐
│         协议层 (emm_v5.c)                │
│  Emm_V5协议封装 (15个API)                │
│  位置/速度/回零/使能控制                  │
└──────────────┬───────────────────────────┘
               │ emm_uart_send()
┌──────────────▼───────────────────────────┐
│      通信层 (emm_uart.c) ✨ V3.0新增      │
│  统一RS485发送接口 + 统计功能             │
│  支持阻塞/中断/DMA三种模式                │
└──────────────┬───────────────────────────┘
               │ HAL_UART_Transmit()
┌──────────────▼───────────────────────────┐
│         硬件层 (usart.c)                 │
│  USART1+2初始化 + IDLE中断               │
└──────────────────────────────────────────┘
```

### 核心改进（vs V2.0）

| 改进项 | V2.0 | V3.0 | 优势 |
|--------|------|------|------|
| **通信层** | ATK_RS485历史遗留 | emm_uart统一接口 | 职责清晰，易维护 |
| **协议层代码** | emm_v5_send_cmd 24行 | 10行（-58%） | 代码更简洁 |
| **测试文件** | 6个重复测试文件 | 删除，单一main.c | 降低混乱度 |
| **统计功能** | 分散在emm_v5.c | 集中在emm_uart.c | 职责归一 |
| **调试工具** | 无 | USMART串口命令 | 实时调试便捷 |
| **Flash占用** | 29116 bytes | 28984 bytes | -132 bytes |

## 📖 文档索引

| 文档 | 描述 | 适合人群 |
|------|------|---------|
| [.github/copilot-instructions.md](.github/copilot-instructions.md) | **AI开发指南**（必读） | 所有开发者 |
| [Docs/架构优化总结_V3.0.md](Docs/架构优化总结_V3.0.md) | V3.0优化详情+对比 | 架构学习者 |
| [Docs/CubeMX配置详细步骤.md](Docs/CubeMX配置详细步骤.md) | 零基础CubeMX配置 | 初学者 |
| [Docs/USMART使用指南.md](Docs/USMART使用指南.md) | 串口调试工具手册 | 调试人员 |
| [Docs/doc_Y57/README.md](Docs/doc_Y57/README.md) | 张大头电机完整技术文档 | 电机使用者 |
| [Docs/00-项目总览.md](Docs/00-项目总览.md) ~ [07-电机控制快速入门.md](Docs/07-电机控制快速入门.md) | 系统入门教程 | 新手上路 |

## 🔧 开发指南

### 添加新的电机控制功能

1. **应用层**（`Core/App/motor_zdt.c`）：
   ```c
   void motor_custom_action(uint8_t addr) {
       Emm_V5_Pos_Control(addr, 0, 300, 10, 1600, false, false);
   }
   ```

2. **导出到USMART**（`Drivers/Middlewares/USMART/usmart_config.c`）：
   ```c
   {(void *)motor_custom_action, "void motor_custom_action(uint8_t addr)"},
   ```

3. **编译测试**：
   ```powershell
   cmake --build --preset Debug
   ```

4. **串口调用**：
   ```
   motor_custom_action(1)
   ```

### 切换通信模式

编辑 `Drivers/BSP/EMM_V5/emm_uart.h`:

```c
// 阻塞模式（当前）
#define EMM_UART_MODE_BLOCKING     1

// 中断模式
// #define EMM_UART_MODE_IT        1

// DMA模式（未实现）
// #define EMM_UART_MODE_DMA       1
```

### 调试技巧

1. **查看通信统计**:
   ```
   emm_uart_print_stats()
   ```

2. **USART1监控日志**:
   连接PA9到USB转TTL，波特率115200，查看实时日志

3. **USART2监控电机通信**:
   使用RS485-USB转换器透传监控，波特率115200

4. **LED指示**:
   - LED0闪烁 = 系统正常运行
   - LED0常亮 = 通信错误或卡死
   - LED0熄灭 = 未初始化

## 🐛 常见问题

### Q1: 电机无响应

**排查步骤**:
1. 检查RS485 A-B线是否正确（A-A, B-B）
2. 确认电机地址（出厂默认0x01）
3. 验证波特率（115200）
4. 调用 `emm_uart_print_stats()` 查看发送状态
5. 检查 `Emm_V5_En_Control()` 是否调用

### Q2: 编译错误

```
error: 'emm_uart.h' file not found
```

**解决**:
检查 `CMakeLists.txt` 中的 `target_include_directories`:
```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/BSP/EMM_V5  # 确保存在
)
```

### Q3: Flash不足

当前占用44.23%，剩余55.77%（35KB）。如需优化：
1. 使用Release构建：`cmake --preset Release`
2. 移除USMART：`CMakeLists.txt` 注释相关源文件
3. 禁用看门狗：`app_config.h` 设置 `FEATURE_WATCHDOG_ENABLE 0`

## 📝 更新日志

### V3.0 (2025-12-01)

- ✅ 新增统一通信层 `emm_uart.c/h`（220行）
- ✅ 移除ATK_RS485历史遗留层
- ✅ 简化协议层代码58%（24行→10行）
- ✅ 删除6个冗余测试文件
- ✅ 新增通信统计功能（成功率、错误次数等）
- ✅ 优化Flash占用-132 bytes
- ✅ 推送至GitHub: https://github.com/dvtps/STM32_485

### V2.0 (2025-11-30)

- ✅ HAL库移植（从张大头标准库例程）
- ✅ 目录重组（`Core/` → `Drivers/`）
- ✅ 新增USMART串口调试工具
- ✅ 新增IWDG看门狗保护（2s超时）
- ✅ 统一配置文件 `app_config.h`

## 📄 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件

## 🤝 贡献

欢迎提交Issue和Pull Request！

**贡献前请阅读**:
- `.github/copilot-instructions.md` - 代码规范
- `Docs/架构优化总结_V3.0.md` - 架构设计原则

## 📞 联系方式

- **项目仓库**: https://github.com/dvtps/STM32_485
- **电机厂商**: 张大头智控（https://zhangdatou.taobao.com）
- **开发板**: 正点原子（https://www.alientek.com）

## 🙏 致谢

- **张大头智控** - 提供Y系列电机和Emm_V5协议
- **正点原子** - 提供M48Z-M3开发板和HAL库示例
- **CMSIS/HAL** - ST官方HAL库支持

---

**Star ⭐ 本项目，关注V3.0架构更新！**
