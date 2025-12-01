# STM32CubeMX 配置详细操作指南
## 正点原子 M48Z-M3 开发板 (STM32F103C8)

**适用人群**: 零基础/初学者  
**目标**: 生成可用于 VS Code 的 CMake 工程（或 Makefile 工程）  
**预计时间**: 15-20 分钟

---

## 📋 开始前的准备

### 您需要的信息

根据 `STM32F103最小系统板IO引脚分配表.pdf`，我们要配置的引脚：

| 用途 | 引脚 | 功能 | 备注 |
|------|------|------|------|
| LED指示灯 | **PA8** | GPIO输出 | 低电平点亮 |
| 唤醒按键 | **PA0** | GPIO输入 | 按下=高电平，**需下拉**(源码) |
| 普通按键 | **PC13** | GPIO输入 | 按下=高电平，**需上拉**(源码) |
| 调试串口TX | **PA9** | USART1_TX | 115200波特率 |
| 调试串口RX | **PA10** | USART1_RX | 115200波特率 |
| RS485串口TX | **PA2** | USART2_TX | 115200波特率 |
| RS485串口RX | **PA3** | USART2_RX | 115200波特率 |
| 调试接口 | PA13/PA14 | SWD | 自动配置 |
| 外部晶振 | PD0/PD1 | 8MHz晶振 | 自动配置 |

---

## 🎯 第一步：创建新项目

### 1.1 启动 STM32CubeMX

- 双击打开 **STM32CubeMX** 软件
- 等待启动完成

### 1.2 新建项目

**在主界面上，您会看到几个选项：**

```
┌────────────────────────────────────┐
│  STM32CubeMX 主界面                 │
├────────────────────────────────────┤
│  [Access to MCU Selector] ← 点这个  │
│  [Access to Board Selector]        │
│  [Load Project]                    │
│  [Start My Project from ST Board]  │
└────────────────────────────────────┘
```

👉 **点击**: `Access to MCU Selector` (MCU选择器)

### 1.3 选择芯片型号

**现在您会看到芯片选择界面：**

**左上角搜索框**（非常显眼的位置）:
```
┌──────────────────────────────┐
│  Part Number Search          │
│  [STM32F103C8        ] 🔍    │ ← 在这里输入
└──────────────────────────────┘
```

**操作**:
1. 在搜索框输入: `STM32F103C8`
2. 按 Enter 或等待1秒
3. **下方列表会显示搜索结果**

**结果列表中会有好几个型号，找这个**:
```
┌─────────────────┬──────────┬─────────┐
│ Part Number     │ Package  │ Core    │
├─────────────────┼──────────┼─────────┤
│ STM32F103C8Tx   │ LQFP48   │ Cortex-M3│ ← 选这个
│ STM32F103C8Hx   │ ...      │         │
│ ...             │ ...      │         │
└─────────────────┴──────────┴─────────┘
```

👉 **双击**: `STM32F103C8Tx` 这一行

或者：
1. 单击选中 `STM32F103C8Tx`
2. 点击右上角的 `Start Project` 按钮

**✅ 成功标志**: 出现一个大的芯片引脚图界面

---

## 🎯 第二步：配置系统核心 (SYS)

### 2.1 打开 SYS 配置

**界面说明**:
```
左侧边栏（树状结构）        中间（引脚图）         右侧（配置面板）
┌─────────────────┐       ┌──────────┐         ┌──────────┐
│ Pinout & Config │       │   芯片    │         │   参数    │
│                 │       │   引脚图  │         │   配置    │
│ ⊕ System Core  │←展开这个│          │         │          │
│   - SYS        │←点这个  │          │         │          │
│   - RCC        │       │          │         │          │
│   - ...        │       │          │         │          │
└─────────────────┘       └──────────┘         └──────────┘
```

**操作步骤**:
1. 看左侧边栏
2. 找到 `System Core` (系统核心) 分类
3. 点击 `System Core` 前面的 **⊕** 号展开
4. 点击 `SYS` 

### 2.2 配置 Debug 接口

**右侧配置面板会显示 SYS 的设置**:

```
┌─────────────────────────────────┐
│ SYS Mode and Configuration      │
├─────────────────────────────────┤
│ Mode                            │
│ ┌─────────────────────────────┐ │
│ │ Debug                        │ │
│ │ [Serial Wire      ▼]        │ │ ← 点这里选择
│ └─────────────────────────────┘ │
│                                 │
│ Timebase Source                │
│ [SysTick         ▼]            │
└─────────────────────────────────┘
```

**操作**:
1. 找到 `Debug` 字样
2. 看到下拉框（可能显示 `No Debug` 或其他）
3. 点击下拉框
4. 选择: **`Serial Wire`**

**✅ 检查**: 引脚图上 PA13 和 PA14 应该变成**黄色**

---

## 🎯 第三步：配置时钟源 (RCC)

### 3.1 打开 RCC 配置

**左侧边栏**:
```
┌─────────────────┐
│ ⊖ System Core  │ ← 已展开
│   - SYS        │
│   - RCC        │ ← 点这个
│   - GPIO       │
│   - NVIC       │
└─────────────────┘
```

👉 **点击**: `RCC`

### 3.2 配置外部高速时钟 (HSE)

**右侧配置面板**:

```
┌──────────────────────────────────────┐
│ RCC Mode and Configuration           │
├──────────────────────────────────────┤
│ Mode                                 │
│                                      │
│ High Speed Clock (HSE)               │ ← 找到这里
│ [Disable                    ▼]       │ ← 点击下拉框
│                                      │
│ Low Speed Clock (LSE)                │
│ [Disable                    ▼]       │
└──────────────────────────────────────┘
```

**操作**:
1. 找到 `High Speed Clock (HSE)` 一行
2. 点击下拉框（现在应该显示 `Disable`）
3. 选择: **`Crystal/Ceramic Resonator`** (水晶/陶瓷谐振器)

**✅ 检查**: 引脚图上 PD0-OSC_IN 和 PD1-OSC_OUT 变成**黄色**

**💡 源码依据**: `Drivers/SYSTEM/sys/sys.c` 第112行
```c
rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
rcc_osc_init.HSEState = RCC_HSE_ON;  // 打开HSE
```

---

## 🎯 第四步：配置串口1 (USART1)

### 4.1 打开 USART1 配置

**左侧边栏，向下滚动，找到 `Connectivity` (连接性)**:

```
┌─────────────────┐
│ ⊕ Connectivity │ ← 展开这个
│   - USART1     │ ← 然后点这个
│   - USART2     │
│   - I2C1       │
│   - SPI1       │
│   - ...        │
└─────────────────┘
```

👉 **操作**: 
1. 点击 `Connectivity` 前的 ⊕ 展开
2. 点击 `USART1`

### 4.2 使能 USART1

**右侧配置面板**:

```
┌──────────────────────────────────────┐
│ USART1 Mode and Configuration        │
├──────────────────────────────────────┤
│ Mode                                 │
│ ┌─────────────────────────────┐      │
│ │ ☐ Asynchronous              │      │ ← 勾选这个
│ │ ☐ Synchronous               │      │
│ │ ☐ Single Wire (Half-Duplex) │      │
│ └─────────────────────────────┘      │
└──────────────────────────────────────┘
```

👉 **勾选**: ☑️ `Asynchronous` (异步模式)

### 4.3 配置参数

**勾选后，下方会出现更多选项卡**:

```
┌─────────────────────────────────────────────┐
│ [Parameter Settings] [NVIC Settings] [...]  │ ← 选项卡
├─────────────────────────────────────────────┤
│ Basic Parameters                            │
│ ┌─────────────────────────────────────┐     │
│ │ Baud Rate           [115200]        │     │
│ │ Word Length         [8 Bits ▼]      │     │
│ │ Parity              [None ▼]        │     │
│ │ Stop Bits           [1 ▼]           │     │
│ └─────────────────────────────────────┘     │
└─────────────────────────────────────────────┘
```

**确认参数**（通常默认就是对的）:
- Baud Rate (波特率): **115200**
- Word Length (数据位): **8 Bits**
- Parity (校验): **None**
- Stop Bits (停止位): **1**

### 4.4 使能中断

**点击 `NVIC Settings` 选项卡**:

```
┌─────────────────────────────────────────────┐
│ [Parameter Settings] [NVIC Settings] [...]  │
├─────────────────────────────────────────────┤
│ NVIC Interrupt Table                        │
│ ┌────────────────────────────────────┐      │
│ │ Enabled  │ Interrupt Name           │      │
│ ├──────────┼─────────────────────────┤      │
│ │ ☐       │ USART1 global interrupt │      │ ← 勾选这个
│ └──────────┴─────────────────────────┘      │
└─────────────────────────────────────────────┘
```

👉 **勾选**: ☑️ `USART1 global interrupt`

✅ **检查**: 引脚图上 PA9(TX) 和 PA10(RX) 变成**绿色**

💡 **关于黄色警告**:
- 当同时配置 USART1 和 USART2 后，可能会看到：
  - USART1 的 `Mode` 项标黄
  - USART1 的 `Hardware Flow Control (RS232)` 项标黄
- **这是正常现象**，不影响使用，原因：
  - CubeMX 提示某些引脚可能有复用冲突
  - 我们只用 TX/RX，不用 RTS/CTS 流控，所以安全
- **无需处理**，直接继续下一步

---

## 🎯 第五步：配置串口2 (USART2)

**重复第四步的操作，只是对象换成 USART2**

### 5.1 选择 USART2

**左侧边栏**:
```
┌─────────────────┐
│ ⊖ Connectivity │
│   - USART1     │
│   - USART2     │ ← 点这个
└─────────────────┘
```

### 5.2 配置 USART2

**完全相同的操作**:
1. ☑️ 勾选 `Asynchronous`
2. 确认参数: 115200, 8N1
3. 切换到 `NVIC Settings`
4. ☑️ 勾选 `USART2 global interrupt`

✅ **检查**: 引脚图上 PA2(TX) 和 PA3(RX) 变成**绿色**

⚠️ **再次提醒**: 配置完 USART2 后，USART1 的黄色警告仍然存在，**这是正常的**，不影响功能，直接忽略即可。

---

## 🎯 第六步：配置 GPIO (LED和按键)

### 6.1 配置 LED0 (PA8)

**在中间的芯片引脚图上操作**:

#### 找到 PA8 引脚:
```
      芯片示意图（顶部区域）
      ┌─────────────────┐
   PA8├─11              │ ← 看引脚编号11附近
   PA9├─12              │
  PA10├─13              │
      └─────────────────┘
```

**操作**:
1. **用鼠标点击引脚图上的 PA8**
2. 会弹出一个菜单:
   ```
   ┌──────────────────────┐
   │ Select PA8 signal    │
   ├──────────────────────┤
   │ Reset_State          │
   │ GPIO_Output      ← 选这个
   │ GPIO_EXTI8           │
   │ TIM1_CH1             │
   │ ...                  │
   └──────────────────────┘
   ```
3. 选择: **`GPIO_Output`**

#### 设置引脚标签:
1. **右键点击 PA8** (刚才变绿的那个引脚)
2. 菜单中选择: `Enter User Label` (输入用户标签)
3. 弹出输入框，输入: **`LED0`**
4. 点击 OK

**✅ PA8 现在应该显示为绿色，并标注 "LED0"**

### 6.2 配置 KEY0 (PC13)

**找到 PC13**:
```
      芯片示意图（左上角）
      ┌─────────────────┐
      │              2─┤PC13
      │              3─┤PC14
      │              4─┤PC15
      └─────────────────┘
```

**操作**:
1. 点击 **PC13**
2. 弹出菜单，选择: **`GPIO_Input`** (输入模式)
3. 右键 PC13 → `Enter User Label` → 输入: **`KEY0`**

💡 **注意**: 源码 key.c 第38行配置为**上拉** `GPIO_PULLUP`

### 6.3 配置 KEY_WKUP (PA0)

**找到 PA0**:
```
      芯片示意图（左侧下方）
      ┌─────────────────┐
      │             10─┤PA0-WKUP
      │             11─┤PA1
      └─────────────────┘
```

**操作**:
1. 点击 **PA0**
2. 选择: **`GPIO_Input`**
3. 右键 PA0 → `Enter User Label` → 输入: **`KEY_WKUP`**

💡 **注意**: 源码 key.c 第43行配置为**下拉** `GPIO_PULLDOWN`

### 6.4 详细配置 GPIO 参数

**现在需要配置上拉/下拉电阻**

#### 打开 GPIO 配置:
```
左侧边栏
┌─────────────────┐
│ ⊖ System Core  │
│   - SYS        │
│   - RCC        │
│   - GPIO       │ ← 点这个
└─────────────────┘
```

#### 配置 LED0 (PA8):
**右侧会显示所有 GPIO 列表，找到 PA8**:
```
┌────────────────────────────────────────┐
│ Pin Name: PA8                          │
│ Signal on Pin: GPIO_Output             │
├────────────────────────────────────────┤
│ GPIO mode: [Output Push Pull     ▼]   │ ← 确认这个
│ GPIO Pull-up/Pull-down:               │
│            [No pull-up and no... ▼]   │ ← 选这个
│ Maximum output speed: [Low        ▼]  │
│ User Label: LED0                       │
└────────────────────────────────────────┘
```

**设置**:
- GPIO mode: `Output Push Pull`
- Pull-up/Pull-down: **`No pull-up and no pull-down`**
- Speed: `Low`

#### 配置 KEY0 (PC13):
**滚动找到 PC13**:
```
┌────────────────────────────────────────┐
│ Pin Name: PC13-TAMPER-RTC              │
│ Signal on Pin: GPIO_Input              │
├────────────────────────────────────────┤
│ GPIO mode: [Input mode           ▼]   │
│ GPIO Pull-up/Pull-down:               │
│            [Pull-up              ▼]   │ ← 选上拉
│ User Label: KEY0                       │
└────────────────────────────────────────┘
```

**设置**:
- GPIO mode: `Input mode`
- Pull-up/Pull-down: **`Pull-up`** (遵循源码 key.c 第38行配置)

#### 配置 KEY_WKUP (PA0):
**找到 PA0-WKUP**:
```
┌────────────────────────────────────────┐
│ Pin Name: PA0-WKUP                     │
│ Signal on Pin: GPIO_Input              │
├────────────────────────────────────────┤
│ GPIO mode: [Input mode           ▼]   │
│ GPIO Pull-up/Pull-down:               │
│            [Pull-down            ▼]   │ ← 选下拉
│ User Label: KEY_WKUP                   │
└────────────────────────────────────────┘
```

**设置**:
- GPIO mode: `Input mode`
- Pull-up/Pull-down: **`Pull-down`** (按下时是高电平，所以需要下拉)

**✅ 到此 GPIO 全部配置完成！**

---

## 🎯 第七步：配置时钟树 ⏰

**这是最关键的一步！**

### 7.1 打开时钟配置

**顶部有一排标签页**:
```
[Pinout & Configuration] [Clock Configuration] [...]
                         ↑ 点这个标签
```

👉 **点击**: `Clock Configuration` 标签

### 7.2 认识时钟树界面

**您会看到一个复杂的流程图，从左到右**:
```
[时钟源] → [PLL倍频] → [系统时钟] → [各总线时钟]
```

**不要慌！我们只需要改3个地方！**

### 7.3 确认外部晶振频率

**最左边找到 HSE 框**:
```
┌──────────────────────┐
│   Input frequency    │
│     HSE (MHz)        │
│     [8.0]     ←应该显示8│
│                      │
│   ○ HSE  ←选中这个单选框│
│   ○ HSI              │
└──────────────────────┘
```

**💡 源码依据**: `Users/stm32f1xx_hal_conf.h` 第80行
```c
#define HSE_VALUE    8000000U  // 8MHz外部晶振（物理晶振）
```

**⚠️ 重要区分**:
- **外部晶振 (HSE)**: 8MHz ← 硬件固定，物理晶体频率
- **系统时钟 (SYSCLK)**: 72MHz ← 通过 PLL 倍频得到 (8MHz × 9 = 72MHz)
- 正点原子开发板上的物理晶振是 8MHz，最终系统运行在 72MHz

**操作**:
1. 找到时钟树最左边的 `HSE` 输入框
2. **确认显示为 `8` 或 `8.0`**（通常已经是8MHz，因为第三步配置了Crystal/Ceramic Resonator）
3. 如果不是8，点击输入框改为: **`8`**
4. **重要**：点击 **HSE 旁边的单选按钮** ○，选中 HSE 作为 PLL 时钟源

### 7.4 配置 PLL 倍频

**中间偏左找到 PLL 相关配置**:
```
┌─────────────────────────┐
│   PLL Source Mux        │
│   [HSE           ▼] ←选这个│
└─────────────────────────┘

┌─────────────────────────┐
│       PLLCLK            │
│   [x 9           ▼] ←选x9│
│   = 72 MHz              │
└─────────────────────────┘
```

**操作**:
1. 找到 `PLL Source Mux` 下拉框
2. 选择: **`HSE`**
3. 找到 PLL 倍频系数（通常在 PLLCLK 附近）

**💡 源码依据**: `sys.c` 第115-116行
```c
rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;  // PLL源选HSE
rcc_osc_init.PLL.PLLMUL = plln;  // main.c传入RCC_PLL_MUL9
```
4. 选择: **`x9`** (8MHz × 9 = 72MHz)

### 7.5 设置系统时钟为 72MHz

**最右边找到 HCLK 输入框**:
```
┌──────────────────────────────────┐
│           System Clock Mux       │
│           [PLLCLK        ▼] ←选这个│
└──────────────────────────────────┘

┌──────────────────────────────────┐
│  HCLK (MHz)                      │
│  [72]  ← 直接输入72              │
└──────────────────────────────────┘
```

**操作**:
1. 找到 `System Clock Mux` 下拉框
2. 选择: **`PLLCLK`**
3. 找到最右边的 `HCLK (MHz)` 输入框
4. 点击输入框，输入: **`72`**
5. **按 Enter 键**

**💡 源码依据**: `sys.c` 第126-130行
```c
rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 系统时钟来自PLL
rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB不分频=72MHz
rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV2;          // APB1二分频=36MHz
rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;          // APB2不分频=72MHz
```

### 7.6 让 CubeMX 自动计算

**按 Enter 后，CubeMX 会自动计算所有时钟**

**✅ 检查结果**（右侧应该显示）:
```
SYSCLK = 72 MHz
HCLK   = 72 MHz  
APB1   = 36 MHz (自动设置)
APB2   = 72 MHz (自动设置)
```

**💡 如果出现黄色/红色警告**:
- 黄色: 忽略即可
- 红色: 点击右下角的 `Resolve Clock Issues` 按钮，让 CubeMX 自动修正

---

## 🎯 第八步：项目设置

**这一步决定生成什么类型的工程！**

### 8.1 打开项目管理器

**顶部标签页**:
```
[...] [Clock Configuration] [Project Manager] [...]
                             ↑ 点这个
```

### 8.2 设置项目基本信息

**Project 标签页**（默认就在这个页面）:

```
┌─────────────────────────────────────────┐
│ Project Settings                        │
├─────────────────────────────────────────┤
│ Project Name                            │
│ [STM32_485_VSCode          ]  ← 输入项目名│
│                                         │
│ Project Location                        │
│ [D:\STM32\Projects\ZDT\   ] 📁 ← 选择路径│
│                                         │
│ Toolchain/IDE                           │
│ [CMake                     ▼]  ← 选这个 │
└─────────────────────────────────────────┘
```

**操作**:
1. **Project Name**: 输入 **`STM32_485_VSCode`**
2. **Project Location**: 点击 📁 按钮，选择 `D:\STM32\Projects\ZDT\`
3. **Toolchain/IDE**: 下拉选择 **`CMake`**（推荐）或 **`Makefile`**

**💡 为什么选 CMake？**
- VS Code 原生支持（CMake Tools 插件）
- 跨平台构建系统
- 更现代化的依赖管理
- 与 STM32 Extension for VS Code 完美集成
- 使用开源编译器（arm-none-eabi-gcc）

**📌 如果下拉列表找不到 CMake**:
- 可以选择 **`Makefile`**（功能类似，也能正常工作）
- CMake 需要 CubeMX 6.0+ 版本支持

### 8.3 代码生成设置

**点击上方的 `Code Generator` 标签**:

```
┌────────────────────────────────────────────┐
│ Code Generator                             │
├────────────────────────────────────────────┤
│ STM32Cube MCU packages and embedded       │
│ software packs                             │
│ ○ Copy only the necessary library files   │
│ ● Copy all used libraries into the project│ ← 选这个
└────────────────────────────────────────────┘

Generated files
┌────────────────────────────────────────────┐
│ ☑ Generate peripheral initialization as   │ ← 勾选
│   a pair of '.c/.h' files per peripheral  │
│                                            │
│ ☑ Keep User Code when re-generating       │ ← 勾选（重要！）
│                                            │
│ ☐ Delete previously generated files...    │ ← 不要勾选
└────────────────────────────────────────────┘
```

**勾选项**:
- ☑️ **Copy all used libraries into the project** (复制所有库到项目)
- ☑️ **Generate peripheral initialization as a pair of '.c/.h' files per peripheral** (每个外设生成独立文件)
- ☑️ **Keep User Code when re-generating** (重新生成时保留用户代码，非常重要！)
- ☐ **Delete previously generated files** (不要勾选，避免误删)

---

## 🎯 第九步：生成代码 🎉

**万事俱备，只欠东风！**

### 9.1 最后检查

**快速检查清单** (按 `Alt+Tab` 切换到 Pinout & Configuration 查看):
```
☑ SYS: Debug = Serial Wire (PA13/PA14黄色)
☑ RCC: HSE = Crystal/Ceramic (PD0/PD1黄色)
☑ Clock: 72MHz
☑ USART1: 115200, 中断使能 (PA9/PA10绿色)
☑ USART2: 115200, 中断使能 (PA2/PA3绿色)
☑ PA8: GPIO_Output (LED0, 绿色)
☑ PA0: GPIO_Input, Pull-down (KEY_WKUP, 绿色) ← 按下=高电平
☑ PC13: GPIO_Input, Pull-up (KEY0, 绿色) ← 按下=高电平
```

### 9.2 生成代码

**方法1: 使用菜单**:
```
顶部菜单栏
[Project] → [Generate Code]
   ↑点击这个菜单
```

**方法2: 快捷键**:
- 按 **`Ctrl + Shift + G`**

**方法3: 工具栏按钮**:
- 点击右上角的**齿轮图标** ⚙️ (Generate Code)

### 9.3 等待生成

**会弹出生成进度窗口**:
```
┌─────────────────────────────┐
│ Generating project...       │
│ ▓▓▓▓▓▓▓▓▓░░░░░░░ 60%       │
│ Generating CMakeLists.txt...│
└─────────────────────────────┘
```

⏳ 等待 10-30 秒...

### 9.4 生成完成

**成功后会弹出提示**:
```
┌─────────────────────────────────┐
│ Code generation completed       │
├─────────────────────────────────┤
│ Do you want to open the         │
│ generated folder?               │
│                                 │
│   [Open Folder]  [Open Project] │
└─────────────────────────────────┘
```

👉 **点击**: `Open Folder` (打开文件夹)

**Windows 资源管理器会打开项目目录！**

---

## ✅ 第十步：验证生成的文件

**检查目录结构**:

```
D:\STM32\Projects\ZDT\STM32_485_VSCode\
├── 📁 Core/                   ← 核心代码
│   ├── 📁 Inc/                ← 头文件
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   ├── stm32f1xx_it.h
│   │   ├── usart.h
│   │   └── gpio.h
│   └── 📁 Src/                ← 源文件
│       ├── main.c             ← ✨ 主程序
│       ├── stm32f1xx_it.c
│       ├── stm32f1xx_hal_msp.c
│       ├── usart.c
│       └── gpio.c
├── 📁 Drivers/                ← HAL库驱动
│   ├── 📁 CMSIS/
│   └── 📁 STM32F1xx_HAL_Driver/
├── 📄 CMakeLists.txt          ← ✨ CMake构建文件（或Makefile）
├── 📄 STM32_485_VSCode.ioc    ← CubeMX配置文件
├── 📄 startup_stm32f103c8tx.s ← 启动文件
└── 📄 STM32F103C8Tx_FLASH.ld  ← 链接脚本
```

**✅ 如果看到这些文件，说明生成成功！**

---

## 🎊 恭喜！CubeMX 配置完成！

**您已经完成了**:
- ✅ 芯片选型
- ✅ 系统核心配置（调试接口）
- ✅ 时钟源配置（外部8MHz晶振）
- ✅ 时钟树配置（72MHz系统时钟）
- ✅ 串口配置（USART1/USART2）
- ✅ GPIO配置（LED和按键）
- ✅ 项目设置（CMake/Makefile 工程）
- ✅ 代码生成

---

## 📝 下一步操作

**现在您有两个选择**:

### 选项A: 在 VS Code 中继续开发
**需要**:
1. 安装 arm-none-eabi-gcc 编译器
2. 安装 Make 工具
3. 复制原项目的 BSP 驱动
4. 配置 VS Code

👉 **告诉我**: "我要在 VS Code 中继续开发"，我会详细指导

### 选项B: 在 VS Code 中编译和调试
**在 VS Code 中使用**（推荐）:
1. 用 VS Code 打开生成的项目文件夹
2. 安装必要插件：
   - **CMake Tools** (如果选了CMake)
   - **Cortex-Debug** (调试插件)
3. 按 **Ctrl+Shift+P** → 选择 `CMake: Select Kit` → 选择 arm-none-eabi-gcc
4. 按 **F7** 或点击底部状态栏的 `Build` 按钮编译
5. Debug 模式已在 VS Code 配置中设置好

**或者手动编译测试**:
```powershell
cmake --build build    # 使用已配置的构建类型
```

👉 **告诉我**: "我要配置 VS Code 环境"，我会详细指导插件安装和配置

---

## ❓ 常见问题

### Q1: 找不到芯片型号
**A**: 确保输入的是 `STM32F103C8`，注意大小写

### Q2: 引脚无法点击
**A**: 可能已经被其他功能占用，检查是否有冲突

### Q3: 时钟无法设置为 72MHz
**A**: 
1. 确认 HSE 设置为 8MHz
2. 确认 PLL 选择 HSE 并倍频 x9
3. 点击 "Resolve Clock Issues"

### Q4: 生成代码失败
**A**: 检查项目路径是否有中文或特殊字符

---

## 📞 需要帮助？

**如果遇到问题，请告诉我**:
1. 卡在哪一步了？
2. 看到什么错误信息？
3. 截图会更容易帮您解决！

**现在请告诉我您的进度** 😊
