# STM32F103 步进电机控制系统 - AI 开发指南

## 项目概述

基于STM32F103C8的闭环步进电机控制系统，使用张大头Y系列V2.0电机通过RS485通信。项目采用HAL库+CMake构建，核心为双串口架构：USART1调试输出 + USART2电机通信。

**目标硬件**: 正点原子M48Z-M3开发板（STM32F103C8，72MHz）  
**电机驱动**: 张大头Emm_V5.0协议（16细分，3200脉冲/圈）  
**构建系统**: CMake + gcc-arm-none-eabi + Ninja

---

## 架构要点

### 双串口分工模式（关键设计）

```
USART1 (PA9/PA10)  → printf调试输出, 115200bps, 轮询发送
USART2 (PA2/PA3)   → RS485电机通信, 115200bps, IDLE中断接收
```

**严格分离原则**: 
- USART1仅用于日志（`Drivers/SYSTEM/usart/usart.c`重定向fputc）
- USART2专属电机协议，使用IDLE中断检测帧边界（见`usart2_idle_callback()`）

### 电机通信帧检测机制

**FIFO + IDLE中断双层架构**（`Drivers/BSP/EMM_V5/emm_fifo.c` + `usart.c`）:

1. **RXNE中断**: 字节级入队到环形FIFO（256字节）
2. **IDLE中断**: 触发帧解析，数据从FIFO出队到`g_emm_rx_cmd[]`
3. **标志位同步**: 设置`g_emm_frame_complete=1`，主循环轮询处理

```c
/* 典型响应帧处理流程 (motor_zdt.c) */
if (g_emm_frame_complete) {
    g_emm_frame_complete = 0;
    if (g_emm_rx_count >= 4 && g_emm_rx_cmd[0] == 0x01) {
        uint8_t addr = g_emm_rx_cmd[1];
        uint8_t cmd = g_emm_rx_cmd[2];
        /* 解析电机状态... */
    }
}
```

**为何IDLE**: 电机响应帧长度不定（4-10字节），IDLE中断提供自然的帧分隔符。

### 目录结构语义

```
Core/
└── App/
    ├── main.c             # 主程序入口：初始化+主循环
    ├── motor_zdt.c        # 应用层：按键控制+LED指示，调用Emm_V5_*
    ├── app_config.h       # 统一配置文件：电机/RS485/系统参数
    └── app_functions.h    # USMART可调用函数声明

Drivers/
├── BSP/                   # 板级驱动
│   ├── EMM_V5/            # 电机协议封装（移植自张大头例程）
│   │   ├── emm_v5.c       # 15个API：位置/速度/回零/使能控制
│   │   └── emm_fifo.c     # 环形队列（256字节）
│   ├── LED/               # LED指示灯驱动
│   ├── KEY/               # 按键输入驱动
│   └── IWDG/              # 独立看门狗驱动（2s超时）
├── SYSTEM/                # 核心系统驱动
│   ├── usart/             # USART1+2统一管理，RS485发送+IDLE接收
│   ├── delay/             # 72MHz校准的延时
│   └── sys/               # 时钟配置（RCC_PLL_MUL9 = 8MHz*9）
└── Middlewares/           # 中间件层
    └── USMART/            # 正点原子串口调试组件
        ├── usmart.c       # 命令解析核心
        ├── usmart_config.c # 可调用函数列表配置
        └── usmart_port.c  # TIM4定时扫描+串口接收
```

**命名约定**: 
- 全局变量前缀`g_`（如`g_emm_frame_complete`）
- HAL外设句柄`g_uart1_handle`/`g_uart2_handle`
- FIFO函数前缀`emm_fifo_*`

**通信层简化**: `emm_v5.c`通过`atk_rs485_send_data()`薄封装调用USART2，实际可直接使用`HAL_UART_Transmit(&g_uart2_handle, ...)`（ATK_RS485模块为历史遗留兼容层）。

---

## 关键API与使用模式

### Emm_V5电机控制API（`emm_v5.h`）

**使能电机**（必须首先调用）:
```c
Emm_V5_En_Control(0x01, true, false);  // 地址1, 使能, 不同步
```

**位置模式**（最常用）:
```c
Emm_V5_Pos_Control(
    0x01,       // 电机地址（RS485总线可支持1-255）
    0,          // 方向: 0=CW顺时针, 1=CCW逆时针
    300,        // 速度: 0-5000 RPM
    10,         // 加速度: 0=直接启动, 1-255梯形加减速
    3200,       // 脉冲数: 16细分时3200=1圈, 1600=半圈
    false,      // 相对运动(false) vs 绝对运动(true)
    false       // 立即执行(false) vs 同步等待(true)
);
```

**速度模式**（连续旋转）:
```c
Emm_V5_Vel_Control(0x01, 0, 500, 20, false);  // 500RPM持续转动
Emm_V5_Stop_Now(0x01, false);                 // 急停
```

**多机同步运动**:
```c
Emm_V5_Pos_Control(0x01, 0, 300, 10, 1600, false, true);  // 电机1, snF=true
Emm_V5_Pos_Control(0x02, 1, 300, 10, 1600, false, true);  // 电机2, snF=true
Emm_V5_Synchronous_motion(0);  // 广播地址0，同时启动所有电机
```

### 初始化序列标准模板（`main.c`）

```c
HAL_Init();
MX_GPIO_Init();                     // CubeMX生成的GPIO初始化
sys_stm32_clock_init(RCC_PLL_MUL9); // 72MHz系统时钟
delay_init(72);                     // 延时函数校准
led_init();
key_init();
usart_init(115200);                 // 初始化USART1+2
motor_zdt_init();                   // 应用层初始化
HAL_Delay(100);                     // 等待电机上电

#if FEATURE_WATCHDOG_ENABLE
iwdg_init(4, 1000);                 // 看门狗初始化(2s超时)
#endif

usmart_dev.init(72);                // USMART初始化(系统时钟72MHz)
```

**时钟依赖**: `sys_stm32_clock_init()`必须在`delay_init()`前调用，参数必须匹配（72MHz → 72）。

### USMART串口调试工具（新增V2.0）

**功能**: 通过USART1串口助手调用任意C函数，支持10进制/16进制参数输入。

**使用方法**:
```
? 或 help             # 查看命令列表
motor_enable(1,1)     # 使能电机1
motor_pos_move(1,0,300,10,3200)  # 电机1顺时针转1圈(300RPM,加速度10)
motor_vel_move(1,0,500,20)       # 电机1连续转动500RPM
motor_stop(1)         # 急停电机1
motor_home(1)         # 电机1回零
delay_ms(1000)        # 延时1000ms
```

**添加新函数到USMART**:
1. 在`Core/App/app_functions.h`声明函数
2. 在`Drivers/Middlewares/USMART/usmart_config.c`的`usmart_nametab[]`添加:
   ```c
   {(void *)your_func, "void your_func(uint8_t param1,uint16_t param2)"},
   ```
3. 注意函数签名必须精确匹配，最多支持10个参数

**实现细节**:
- TIM4定时中断（10ms周期）自动扫描USART1接收缓冲区
- 命令格式: `函数名(参数1, 参数2, ...)`，回车结束
- 支持`hex`/`dec`指令切换参数显示格式
- 兼容层: `g_usart_rx_buf`/`g_usart_rx_sta`宏定义指向USART1缓冲区

### 按键控制模式（`motor_zdt.c`实现）

```c
key = key_scan(0);
if (key == KEY0_PRES)         // PC13按键 → 使能电机
if (key == WKUP_PRES)         // PA0按键 → 位置模式测试（1圈）
```

**非阻塞设计**: `key_scan(0)`返回0表示无按键，主循环继续执行。

---

## CMake构建系统

### 标准构建流程

```powershell
cmake --preset Debug           # 配置（生成build/Debug/）
cmake --build --preset Debug   # 构建STM32_485.elf
```

**输出文件**: `build/Debug/STM32_485.elf` + `.map` + `.bin`（通过objcopy生成）

### 添加新源文件

**必须同步修改两处**:
1. **`CMakeLists.txt`**: `target_sources()`添加.c文件
2. **`CMakeLists.txt`**: `target_include_directories()`添加头文件路径

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/BSP/NEW_MODULE/new_driver.c  # 添加这里
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/BSP/NEW_MODULE               # 添加这里
)
```

**避免手动编辑**: `cmake/stm32cubemx/CMakeLists.txt`由STM32CubeMX自动生成，勿修改。

### 工具链配置（`cmake/gcc-arm-none-eabi.cmake`）

- **MCU标志**: `-mcpu=cortex-m3`（STM32F103）
- **链接脚本**: `STM32F103C8Tx_FLASH.ld`（64KB Flash, 20KB RAM）
- **优化等级**: Debug=`-O0 -g3`, Release=`-Os -g0`
- **Nano库**: `--specs=nano.specs`（减小固件体积）

---

## 调试与烧录

### VS Code任务配置（推荐添加）

```json
// .vscode/tasks.json
{
    "label": "Build Debug",
    "type": "shell",
    "command": "cmake --build --preset Debug",
    "problemMatcher": ["$gcc"]
},
{
    "label": "Flash with OpenOCD",
    "type": "shell",
    "command": "openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c 'program build/Debug/STM32_485.elf verify reset exit'",
    "dependsOn": "Build Debug"
}
```

### 串口调试工具

**查看printf输出**: 连接USART1 (PA9/PA10)到USB转TTL模块，115200-8-N-1。

**监控电机通信**: 
- USART2通过RS485模块透传，需RS485-USB转换器
- 可在`usart2_idle_callback()`添加调试打印（注意：会影响实时性）

### 常见问题排查

**电机无响应**:
1. 检查RS485 A-B线是否正确（A-A, B-B）
2. 确认电机地址设置（出厂默认0x01）
3. 验证波特率匹配（115200）
4. 检查`Emm_V5_En_Control()`是否调用

**IDLE中断未触发**:
- 确认`__HAL_UART_ENABLE_IT(&g_uart2_handle, UART_IT_IDLE)`已调用
- 检查`USART2_IRQHandler`是否正确处理IDLE标志（`SR.IDLE`位）

**编译错误找不到头文件**:
- 检查`CMakeLists.txt`的`target_include_directories()`
- 重新运行`cmake --preset Debug`清除缓存

---

## 项目扩展指南

### 添加新的BSP驱动（遵循现有模式）

1. 在`Drivers/BSP/NEW_DRIVER/`创建`new_driver.c/h`
2. 头文件定义GPIO宏:
   ```c
   #define NEW_GPIO_PORT           GPIOB
   #define NEW_GPIO_PIN            GPIO_PIN_5
   #define NEW_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)
   ```
3. 实现`new_driver_init()`，在`main.c`的`USER CODE BEGIN 2`调用
4. 更新`CMakeLists.txt`

### 多电机控制扩展

**地址分配策略**: 
- 修改电机地址：通过上位机或拨码开关（参见`Docs/doc_Y57/06-多机通讯.md`）
- 推荐1-8号连续编号

**同步运动示例**（两轴联动）:
```c
/* 阶段1: 发送同步命令 */
Emm_V5_Pos_Control(0x01, 0, 300, 10, 3200, false, true);  // X轴
Emm_V5_Pos_Control(0x02, 1, 300, 10, 1600, false, true);  // Y轴

/* 阶段2: 触发同步 */
Emm_V5_Synchronous_motion(0);  // 两轴同时启动

/* 阶段3: 等待完成（轮询FLAG或响应帧） */
HAL_Delay(5000);  // 简单方案，生产代码应检查到位标志
```

### 回零功能（四种模式）

详见`Docs/doc_Y57/05-回零功能.md`和`emm_v5.c`的`Emm_V5_Origin_*`函数族：
- **模式0**: 单圈就近回零（无限位开关）
- **模式1**: 单圈方向回零（无限位开关）
- **模式2**: 多圈无限位回零（失速检测）
- **模式3**: 多圈限位开关回零

**典型用法**:
```c
Emm_V5_Origin_Trigger_Return(0x01, 0, false);  // 电机1，模式0，立即执行
```

---

## 参考文档路径

- **电机完整文档**: `Docs/doc_Y57/README.md`（张大头Y系列技术手册）
- **STM32集成指南**: `Docs/doc_Y57/07-STM32集成指南.md`（HAL库移植详解）
- **CubeMX配置**: `Docs/CubeMX配置详细步骤.md`（重新生成工程的步骤）
- **参考例程**: `Docs/STM32_ZDT_PosMode/`（标准库版本，可对比学习）

---

## 代码风格约定

- 中文注释为主（继承正点原子风格）
- Doxygen格式函数头（`@brief`, `@param`, `@retval`）
- 宏定义全大写，函数蛇形命名（`snake_case`）
- 全局变量前缀`g_`，静态函数前缀`static`
- 避免在中断中使用`printf`（USART1轮询发送会阻塞）

---

## 修改禁区（避免破坏系统）

- **勿动**: `cmake/stm32cubemx/CMakeLists.txt`（CubeMX自动生成）
- **慎改**: `STM32F103C8Tx_FLASH.ld`（除非调整内存布局）
- **保持**: `emm_v5.c`的API签名（保证与电机协议兼容）
- **注意**: `usart.c`的`USART2_IRQHandler`（IDLE中断关键路径）

## 已完成优化

### V2.0 架构优化（2025-12-01）

1. **移除ATK_RS485冗余层**: `emm_v5.c`直接调用`HAL_UART_Transmit(&g_uart2_handle, ...)`
2. **目录重组**: `Core/SYSTEM` → `Drivers/SYSTEM`, `Core/BSP` → `Drivers/BSP`
3. **统一配置**: 创建`Core/App/app_config.h`集中管理电机/RS485/系统参数
4. **看门狗保护**: 新增`Drivers/BSP/IWDG/`独立看门狗驱动（2s超时）
5. **USMART集成**: 新增串口调试组件，支持通过USART1调用电机控制函数

### 资源占用（V2.0）

```
Flash: 24972 bytes (38.10% of 64KB)
RAM:   3336 bytes (16.29% of 20KB)
```

---

*此文档基于 2025-12-01 代码快照生成，与实际代码保持同步*
