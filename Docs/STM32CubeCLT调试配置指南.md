# STM32CubeCLT调试与烧录配置指南

## 已安装工具

您已安装：**STM32CubeCLT v1.19.0**（包含所有必要工具）

安装路径：`C:\ST\STM32CubeCLT_1.19.0`

---

## 1. 快速烧录（无需调试）

### 方法1: VS Code任务（推荐）

1. 按 `Ctrl+Shift+P`
2. 输入 "Tasks: Run Task"
3. 选择 **"Flash with STM32CubeProgrammer"**

### 方法2: 命令行

```powershell
# 确保CMSIS-DAP已连接并接线正确
C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=SWD -w build/Debug/STM32_485.elf -v -rst
```

**参数说明**:
- `-c port=SWD`: 使用SWD接口连接
- `-w <file>`: 写入ELF/BIN/HEX文件
- `-v`: 烧录后验证
- `-rst`: 烧录完成后复位

---

## 2. 配置VS Code调试（需要Cortex-Debug扩展）

### 2.1 安装Cortex-Debug扩展

1. 打开VS Code扩展面板 (`Ctrl+Shift+X`)
2. 搜索 **"Cortex-Debug"**
3. 安装由 **marus25** 开发的扩展
4. 重启VS Code

### 2.2 硬件连接

```
CMSIS-DAP调试器 → STM32F103C8
────────────────────────────────
SWDIO  → PA13
SWCLK  → PA14
GND    → GND
3.3V   → 3.3V (可选，给目标板供电)
```

**重要检查**:
- ✅ BOOT0跳线接GND（Flash启动模式）
- ✅ 3.3V供电正常（LED灯亮）
- ✅ CMSIS-DAP驱动已安装

### 2.3 验证连接

```powershell
# 测试能否检测到芯片
C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=SWD -l
```

**预期输出**:
```
Device ID   : 0x0410
Device name : STM32F103C8
Flash size  : 64 KBytes
```

**如果报错 "No debug probe detected"**:
1. 检查USB线缆连接
2. 重新插拔CMSIS-DAP
3. 检查设备管理器中是否识别为USB设备
4. 可能需要安装WinUSB驱动（使用Zadig工具）

---

## 3. 开始调试

### 3.1 启动调试会话

1. 在VS Code中按 `F5`
2. 或点击侧边栏的"调试"图标
3. 选择 **"CMSIS-DAP Debug (stlink-server)"**
4. 程序会自动：
   - 编译最新代码
   - 烧录到芯片
   - 停在main函数入口

### 3.2 调试快捷键

| 快捷键 | 功能 |
|--------|------|
| `F5` | 继续运行 |
| `F9` | 设置/取消断点 |
| `F10` | 单步跳过 (Step Over) |
| `F11` | 单步进入 (Step Into) |
| `Shift+F11` | 单步跳出 (Step Out) |
| `Ctrl+Shift+F5` | 重启调试 |
| `Shift+F5` | 停止调试 |

### 3.3 调试面板功能

- **变量窗口**: 查看局部变量和全局变量
- **监视窗口**: 添加表达式监视（如 `g_emm_frame_complete`）
- **调用堆栈**: 查看函数调用链
- **外设寄存器**: 查看STM32外设状态（需SVD文件支持）

---

## 4. 常用任务

### 4.1 编译项目
```powershell
cmake --build --preset Debug
```
或在VS Code中：`Ctrl+Shift+B`

### 4.2 清理重编译
```powershell
cmake --build --preset Debug --clean-first
```

### 4.3 擦除整个芯片
```powershell
# 使用VS Code任务
Ctrl+Shift+P → "Tasks: Run Task" → "Erase Chip"

# 或命令行
C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=SWD -e all
```

### 4.4 查看芯片信息
```powershell
C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=SWD -l
```

---

## 5. 当前项目调试配置

### 串口分配
- **USART1 (PA9/PA10)**: 
  - 用途：printf调试输出
  - 连接：可选，使用USB-TTL查看日志
  - 波特率：115200
  
- **USART2 (PA2/PA3)**: 
  - 用途：RS485通信
  - 连接：ATK-MB024 → CH340 → 串口助手
  - 波特率：115200
  - 编码：UTF-8

### SWD调试接口
- **PA13/PA14**: CMSIS-DAP调试器
- 用于下载程序和实时调试

---

## 6. 故障排除

### 6.1 无法检测到芯片
```
Error: No debug probe detected
```
**解决步骤**:
1. 检查物理连接（SWDIO/SWCLK/GND）
2. 确认供电正常（3.3V）
3. 重新插拔CMSIS-DAP
4. 检查Windows设备管理器
5. 尝试安装WinUSB驱动（Zadig工具）

### 6.2 连接超时
```
Error: Data read failed
Target voltage: ABSENT!
```
**解决步骤**:
1. 检查3.3V供电
2. 确认BOOT0=GND
3. 测量PA13/PA14引脚是否短路
4. 尝试擦除芯片后重试

### 6.3 烧录后程序不运行
**可能原因**:
1. BOOT0跳线位置错误（应接GND）
2. 选项字节配置错误
3. 程序死在初始化阶段

**调试方法**:
- 使用F5启动调试，单步执行检查卡在哪里
- 检查时钟配置是否正确
- 查看LED是否闪烁

---

## 7. 下一步

1. ✅ **安装Cortex-Debug扩展**
2. ✅ **连接CMSIS-DAP硬件** (PA13/PA14)
3. ✅ **测试烧录功能**：`Ctrl+Shift+P` → "Flash with STM32CubeProgrammer"
4. ✅ **启动调试会话**：按 `F5`
5. ✅ **设置断点测试**：在main.c的while循环中点击行号设置断点

---

## 8. 推荐工作流程

1. **开发阶段**: 使用CMSIS-DAP调试（F5启动，设置断点单步执行）
2. **功能测试**: 查看USART1的printf输出（可选）
3. **RS485测试**: 通过USART2 + CH340模块与串口助手通信
4. **最终验证**: 独立供电运行，通过RS485查看实际运行状态

现在请执行第3步测试烧录功能！
