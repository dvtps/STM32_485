# 05 - CMake构建系统详解

> **难度**: ⭐⭐⭐☆☆  
> **预计时间**: 40分钟  
> **前置要求**: 了解基本的Make/Makefile概念

---

## 📋 目录

1. [CMake项目结构](#1-cmake项目结构)
2. [构建流程详解](#2-构建流程详解)
3. [添加源文件](#3-添加源文件)
4. [编译选项配置](#4-编译选项配置)
5. [VS Code任务集成](#5-vs-code任务集成)

---

## 1. CMake项目结构

### 1.1 核心配置文件

```
STM32_485/
├── CMakeLists.txt              # ⭐ 主构建脚本
├── CMakePresets.json           # 预设配置（Debug/Release）
├── STM32F103C8Tx_FLASH.ld      # 链接脚本（内存布局）
├── cmake/
│   ├── gcc-arm-none-eabi.cmake # ⭐ 工具链配置
│   └── stm32cubemx/
│       └── CMakeLists.txt      # CubeMX自动生成（勿修改）
└── build/
    └── Debug/                  # 构建输出目录
        ├── STM32_485.elf       # 可执行文件
        ├── STM32_485.bin       # 二进制固件
        └── STM32_485.map       # 内存映射文件
```

---

### 1.2 CMakeLists.txt 结构分析

**文件位置**: `CMakeLists.txt` (项目根目录)

```cmake
# ========== 第1部分：项目基本信息 ==========
cmake_minimum_required(VERSION 3.22)
project(STM32_485 C ASM)

# ========== 第2部分：工具链配置 ==========
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

# ========== 第3部分：包含CubeMX生成的配置 ==========
add_subdirectory(cmake/stm32cubemx)

# ========== 第4部分：添加用户源文件 ==========
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 应用层代码
    Core/App/main.c
    Core/App/motor_zdt.c
    
    # 系统驱动
    Drivers/SYSTEM/usart/usart.c
    Drivers/SYSTEM/delay/delay.c
    Drivers/SYSTEM/sys/sys.c
    
    # BSP驱动
    Drivers/BSP/LED/led.c
    Drivers/BSP/KEY/key.c
    Drivers/BSP/EMM_V5/emm_v5.c
    Drivers/BSP/EMM_V5/emm_fifo.c
    Drivers/BSP/IWDG/iwdg.c
    
    # 中间件
    Drivers/Middlewares/USMART/usmart.c
    Drivers/Middlewares/USMART/usmart_str.c
    Drivers/Middlewares/USMART/usmart_config.c
    Drivers/Middlewares/USMART/usmart_port.c
)

# ========== 第5部分：添加头文件搜索路径 ==========
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/App
    Drivers/SYSTEM/usart
    Drivers/SYSTEM/delay
    Drivers/SYSTEM/sys
    Drivers/BSP/LED
    Drivers/BSP/KEY
    Drivers/BSP/EMM_V5
    Drivers/BSP/IWDG
    Drivers/Middlewares/USMART
)

# ========== 第6部分：编译选项 ==========
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall           # 开启所有警告
    -Wextra         # 额外警告
    -Wpedantic      # 严格标准检查
    -Wshadow        # 变量遮蔽警告
)

# ========== 第7部分：链接选项 ==========
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map  # 生成.map文件
)

# ========== 第8部分：后处理（生成.bin文件） ==========
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Generating ${PROJECT_NAME}.bin"
)
```

---

### 1.3 工具链配置文件

**文件位置**: `cmake/gcc-arm-none-eabi.cmake`

```cmake
# ========== 工具链路径配置 ==========
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 工具链前缀
set(TOOLCHAIN_PREFIX arm-none-eabi-)

# 指定编译器
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# ========== MCU特定参数 ==========
set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb")

# ========== 编译标志 ==========
set(CMAKE_C_FLAGS "${MCU_FLAGS} -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --specs=nano.specs")

# Debug编译选项
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -DDEBUG")

# Release编译选项
set(CMAKE_C_FLAGS_RELEASE "-Os -g0 -DNDEBUG")

# ========== 链接标志 ==========
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nosys.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")

# 链接数学库和标准C库
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
```

**关键编译选项说明**：

| 选项 | 作用 |
|------|------|
| `-mcpu=cortex-m3` | 指定CPU架构为Cortex-M3 |
| `-mthumb` | 使用Thumb指令集（16位+32位混合） |
| `-fdata-sections` | 每个数据放入独立section |
| `-ffunction-sections` | 每个函数放入独立section |
| `-Wl,--gc-sections` | 链接时删除未使用的section（减小固件体积） |
| `--specs=nano.specs` | 使用Newlib Nano库（更小的C库） |
| `-O0` | Debug模式：无优化 |
| `-Os` | Release模式：优化代码体积 |
| `-g3` | 生成最详细的调试信息 |

---

### 1.4 CMakePresets.json

**文件位置**: `CMakePresets.json`

```json
{
    "version": 3,
    "cmakeMinimumRequired": {
        "major": 3,
        "minor": 22,
        "patch": 0
    },
    "configurePresets": [
        {
            "name": "Debug",
            "displayName": "Debug Configuration",
            "description": "Debug build with full symbols",
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build/Debug",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug",
                "CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
            }
        },
        {
            "name": "Release",
            "displayName": "Release Configuration",
            "description": "Optimized release build",
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build/Release",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Release",
                "CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
            }
        }
    ],
    "buildPresets": [
        {
            "name": "Debug",
            "configurePreset": "Debug"
        },
        {
            "name": "Release",
            "configurePreset": "Release"
        }
    ]
}
```

**作用**：
- 定义多个构建配置（Debug/Release）
- 自动生成`compile_commands.json`（用于IntelliSense）
- 简化命令行操作（`cmake --preset Debug`）

---

## 2. 构建流程详解

### 2.1 完整构建命令

```powershell
# 第1步：配置项目（生成构建文件）
cmake --preset Debug

# 输出示例：
# -- The C compiler identification is GNU 13.2.1
# -- Detecting C compiler ABI info
# -- Detecting C compiler ABI info - done
# -- Configuring done
# -- Generating done
# -- Build files have been written to: D:/STM32/Projects/ZDT/STM32_485/build/Debug

# 第2步：编译项目
cmake --build --preset Debug

# 输出示例：
# [1/45] Building C object CMakeFiles/STM32_485.dir/Core/Src/main.c.obj
# [2/45] Building C object CMakeFiles/STM32_485.dir/Core/Src/gpio.c.obj
# ...
# [45/45] Linking C executable STM32_485.elf
# Memory region         Used Size  Region Size  %age Used
#              RAM:        4632 B        20 KB     22.62%
#            FLASH:       31092 B        64 KB     47.44%

# 第3步：查看固件信息
arm-none-eabi-size build/Debug/STM32_485.elf

# 输出：
#    text    data     bss     dec     hex filename
#   30540     552    4080   35172    8964 build/Debug/STM32_485.elf
```

**输出文件说明**：

| 文件 | 用途 | 包含内容 |
|------|------|---------|
| `.elf` | 可执行文件 | 代码 + 数据 + 调试符号 |
| `.bin` | 纯二进制固件 | 仅代码和数据（用于量产烧录） |
| `.map` | 内存映射文件 | 符号地址表、内存占用详情 |
| `compile_commands.json` | IDE配置 | 编译参数（供IntelliSense使用） |

---

### 2.2 内存分析

**查看.map文件**（`build/Debug/STM32_485.map`）：

```
Memory Configuration

Name             Origin             Length             Attributes
RAM              0x20000000         0x00005000         xrw
FLASH            0x08000000         0x00010000         xr
*default*        0x00000000         0xffffffff

Linker script and memory map

.text           0x08000000    0x7734
 *(.isr_vector)
 .isr_vector    0x08000000      0x130 CMakeFiles/STM32_485.dir/startup_stm32f103xb.s.obj
                0x08000000                g_pfnVectors
 *(.text)
 .text          0x08000130      0x3d0 CMakeFiles/STM32_485.dir/Core/Src/main.c.obj
                0x08000130                main
                0x080002b8                Error_Handler
                0x080002cc                HAL_UART_MspInit
```

**关键段（Section）**：

| 段名 | 位置 | 内容 |
|------|------|------|
| `.isr_vector` | FLASH起始地址 | 中断向量表 |
| `.text` | FLASH | 代码段（函数代码） |
| `.rodata` | FLASH | 只读数据（const变量、字符串） |
| `.data` | RAM（初始值在FLASH） | 已初始化全局变量 |
| `.bss` | RAM | 未初始化全局变量（清零） |
| `.heap` | RAM | 动态内存分配区 |
| `.stack` | RAM顶部 | 函数调用栈 |

**内存占用计算**：
```
Flash占用 = .text + .rodata + .data初始值
RAM占用   = .data + .bss + .heap + .stack
```

---

### 2.3 构建缓存清理

```powershell
# 完全清理（删除build目录）
Remove-Item -Recurse -Force build

# 重新配置
cmake --preset Debug

# 清理特定配置
Remove-Item -Recurse -Force build/Debug

# 仅清理编译产物（保留CMake缓存）
cmake --build --preset Debug --target clean
```

---

## 3. 添加源文件

### 3.1 添加新的C文件

**场景**：创建新的驱动模块 `Drivers/BSP/SPI_Flash/spi_flash.c`

**步骤1**：创建文件

```c
// Drivers/BSP/SPI_Flash/spi_flash.c
#include "spi_flash.h"

void spi_flash_init(void)
{
    // 初始化代码
}

uint8_t spi_flash_read_id(void)
{
    // 读取ID
    return 0xEF;  // W25Q64
}
```

```c
// Drivers/BSP/SPI_Flash/spi_flash.h
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "sys.h"

void spi_flash_init(void);
uint8_t spi_flash_read_id(void);

#endif
```

**步骤2**：修改CMakeLists.txt

在 `target_sources()` 中添加源文件：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ... 已有文件 ...
    
    # 新添加的SPI Flash驱动
    Drivers/BSP/SPI_Flash/spi_flash.c
)
```

在 `target_include_directories()` 中添加头文件路径：

```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # ... 已有路径 ...
    
    # 新添加的头文件路径
    Drivers/BSP/SPI_Flash
)
```

**步骤3**：重新配置和编译

```powershell
# 方法1：仅重新编译（CMake自动检测更改）
cmake --build --preset Debug

# 方法2：强制重新配置
cmake --preset Debug
cmake --build --preset Debug
```

---

### 3.2 添加汇编文件

**场景**：添加优化的数学运算汇编代码

```asm
; Drivers/BSP/Math/fast_sqrt.s
    AREA    FastMath, CODE, READONLY
    EXPORT  fast_sqrt_asm

fast_sqrt_asm PROC
    ; 输入：R0 = uint32_t input
    ; 输出：R0 = uint32_t result
    VSQRT.F32   S0, S0
    BX          LR
    ENDP
    END
```

**修改CMakeLists.txt**：

```cmake
# 添加汇编文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/BSP/Math/fast_sqrt.s
)

# C文件中声明外部汇编函数
# extern uint32_t fast_sqrt_asm(uint32_t input);
```

---

### 3.3 条件编译

**场景**：根据配置选择性编译USB或CAN模块

**CMakeLists.txt**：

```cmake
# 定义配置选项
option(ENABLE_USB "Enable USB support" ON)
option(ENABLE_CAN "Enable CAN support" OFF)

# 条件添加源文件
if(ENABLE_USB)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Drivers/BSP/USB/usb_device.c
        Drivers/BSP/USB/usbd_cdc.c
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_USB)
endif()

if(ENABLE_CAN)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Drivers/BSP/CAN/can.c
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_CAN)
endif()
```

**C代码中使用**：

```c
#ifdef USE_USB
    #include "usb_device.h"
    void usb_init(void) { /* ... */ }
#endif

#ifdef USE_CAN
    #include "can.h"
    void can_init(void) { /* ... */ }
#endif
```

---

## 4. 编译选项配置

### 4.1 添加宏定义

**场景**：定义调试级别、版本号

```cmake
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # STM32芯片型号
    STM32F103xB
    USE_HAL_DRIVER
    
    # 调试级别
    DEBUG_LEVEL=2
    
    # 版本信息
    FW_VERSION_MAJOR=2
    FW_VERSION_MINOR=0
    FW_VERSION_PATCH=1
    
    # 功能开关
    FEATURE_WATCHDOG_ENABLE=1
    FEATURE_USMART_ENABLE=1
)
```

**C代码中使用**：

```c
#ifndef FW_VERSION_MAJOR
    #define FW_VERSION_MAJOR 0
#endif

void print_version(void)
{
    printf("Firmware Version: %d.%d.%d\r\n",
           FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
}

#if FEATURE_WATCHDOG_ENABLE
    iwdg_init(4, 1000);
#endif
```

---

### 4.2 优化级别对比

| 级别 | 选项 | 代码体积 | 执行速度 | 调试难度 | 适用场景 |
|------|------|---------|---------|---------|---------|
| **-O0** | 无优化 | 最大 | 最慢 | 容易 | 开发调试 ⭐ |
| **-O1** | 基础优化 | 中等 | 中等 | 中等 | 快速测试 |
| **-O2** | 标准优化 | 较小 | 较快 | 困难 | 功能测试 |
| **-O3** | 激进优化 | 可能增大 | 最快 | 很困难 | 性能关键代码 |
| **-Os** | 体积优化 | 最小 | 较快 | 困难 | 生产发布 ⭐ |

**示例配置**：

```cmake
# Debug模式：-O0 便于调试
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -DDEBUG")

# Release模式：-Os 减小固件体积
set(CMAKE_C_FLAGS_RELEASE "-Os -g0 -DNDEBUG")

# 自定义优化模式：-O2 平衡性能和体积
set(CMAKE_C_FLAGS_PERFORMANCE "-O2 -g1")
```

---

### 4.3 警告等级控制

```cmake
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    # 基础警告
    -Wall                   # 常见警告
    -Wextra                 # 额外警告
    
    # 严格检查
    -Wpedantic              # 严格遵守C标准
    -Wshadow                # 变量遮蔽
    -Wdouble-promotion      # float自动提升为double
    -Wconversion            # 隐式类型转换
    -Wcast-align            # 指针对齐问题
    
    # 禁用特定警告
    -Wno-unused-parameter   # 允许未使用的参数（回调函数常见）
    
    # 将警告视为错误（严格模式）
    # -Werror
)
```

---

## 5. VS Code任务集成

### 5.1 tasks.json配置

**文件位置**: `.vscode/tasks.json`

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build Debug (STM32_485)",
            "type": "shell",
            "command": "cmake",
            "args": ["--build", "--preset", "Debug"],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": ["$gcc"],
            "presentation": {
                "reveal": "always",
                "panel": "dedicated"
            }
        },
        {
            "label": "Clean Build",
            "type": "shell",
            "command": "Remove-Item",
            "args": ["-Recurse", "-Force", "build"],
            "problemMatcher": []
        },
        {
            "label": "Flash with PyOCD",
            "type": "shell",
            "command": "pyocd",
            "args": [
                "flash",
                "build/Debug/STM32_485.elf",
                "--target", "stm32f103c8"
            ],
            "dependsOn": "Build Debug (STM32_485)",
            "problemMatcher": []
        },
        {
            "label": "Size Info",
            "type": "shell",
            "command": "arm-none-eabi-size",
            "args": ["build/Debug/STM32_485.elf"],
            "problemMatcher": []
        }
    ]
}
```

**使用方法**：
- `Ctrl+Shift+B` → 选择 "Build Debug (STM32_485)"
- `Ctrl+Shift+P` → "Tasks: Run Task" → 选择任务

---

### 5.2 快捷键绑定

**文件位置**: `.vscode/keybindings.json`

```json
[
    {
        "key": "ctrl+shift+b",
        "command": "workbench.action.tasks.build"
    },
    {
        "key": "ctrl+shift+f",
        "command": "workbench.action.tasks.runTask",
        "args": "Flash with PyOCD"
    },
    {
        "key": "f5",
        "command": "workbench.action.debug.start",
        "when": "!inDebugMode"
    }
]
```

---

### 5.3 构建输出分析

**成功编译输出**：

```
[45/45] Linking C executable STM32_485.elf
Memory region         Used Size  Region Size  %age Used
             RAM:        4632 B        20 KB     22.62%
           FLASH:       31092 B        64 KB     47.44%
Generating STM32_485.bin
```

**错误输出示例**：

```
[15/45] Building C object CMakeFiles/STM32_485.dir/Core/App/main.c.obj
FAILED: CMakeFiles/STM32_485.dir/Core/App/main.c.obj
D:/STM32/Projects/ZDT/STM32_485/Core/App/main.c:42:5: error: 'delay_init' was not declared in this scope
   42 |     delay_init(72);
      |     ^~~~~~~~~~
```

**解决方法**：
1. 检查头文件是否包含：`#include "delay.h"`
2. 检查CMakeLists.txt是否添加了 `Drivers/SYSTEM/delay`

---

## 🎯 下一步

掌握CMake构建系统后，继续学习：

- **[06-调试技巧与问题排查](./06-调试技巧与问题排查.md)** - 综合调试方法和常见问题解决

---

**返回**: [00-项目总览](./00-项目总览.md) | **上一篇**: [04-时钟系统配置](./04-时钟系统配置.md)
