# CMSIS-DAP调试配置指南

## 1. 安装必要工具

### 1.1 安装Cortex-Debug扩展
在VS Code中按 `Ctrl+Shift+X`，搜索并安装：
- **Cortex-Debug** (by marus25)

### 1.2 安装OpenOCD

**方法1: 使用xPack (推荐)**
```powershell
# 安装xpm包管理器
npm install --global xpm@latest

# 安装OpenOCD
xpm install --global @xpack-dev-tools/openocd@latest

# 添加到PATH (示例路径，实际路径可能不同)
# C:\Users\你的用户名\AppData\Roaming\xPacks\@xpack-dev-tools\openocd\0.12.0-4.1\.content\bin
```

**方法2: 手动下载**
1. 下载: https://github.com/xpack-dev-tools/openocd-xpack/releases
2. 解压到 `C:\OpenOCD`
3. 添加 `C:\OpenOCD\bin` 到系统PATH环境变量

**方法3: 使用STM32CubeIDE自带的OpenOCD**
如果已安装STM32CubeIDE，OpenOCD路径通常为：
```
C:\ST\STM32CubeIDE_1.XX.X\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_X.X.X\tools\bin
```
添加此路径到系统PATH。

### 1.3 验证安装
```powershell
openocd --version
```
应该看到版本信息，如：`Open On-Chip Debugger 0.12.0`

---

## 2. 硬件连接

### CMSIS-DAP连接
```
CMSIS-DAP → STM32F103C8
─────────────────────────
SWDIO  → PA13 (SWDIO)
SWCLK  → PA14 (SWCLK)
GND    → GND
3V3    → 3.3V (可选，给目标板供电)
```

**重要**: 确保BOOT0跳线接GND（Flash启动模式）

---

## 3. VS Code调试配置

已配置完成，可在调试面板选择：
- **CMSIS-DAP Debug**: 下载并调试
- **CMSIS-DAP Attach**: 连接到正在运行的程序

---

## 4. 使用方法

### 4.1 启动调试
1. 按 `F5` 或点击调试面板的 "CMSIS-DAP Debug"
2. 程序会自动编译、下载、停在main入口

### 4.2 常用快捷键
- `F5`: 继续运行
- `F10`: 单步跳过 (Step Over)
- `F11`: 单步进入 (Step Into)
- `Shift+F11`: 单步跳出 (Step Out)
- `Shift+F5`: 停止调试
- `Ctrl+Shift+F5`: 重启调试

### 4.3 手动烧录（不启动调试）
```powershell
# 方法1: 使用VS Code任务
Ctrl+Shift+P → "Tasks: Run Task" → "Flash with OpenOCD (CMSIS-DAP)"

# 方法2: 命令行
openocd -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg -c "program build/Debug/STM32_485.elf verify reset exit"
```

---

## 5. 常见问题

### 5.1 OpenOCD找不到设备
```
Error: unable to find a matching CMSIS-DAP device
```
**解决**:
1. 检查CMSIS-DAP驱动是否安装（Windows需要WinUSB驱动）
2. 确认硬件连接正确
3. 尝试重新插拔CMSIS-DAP

### 5.2 连接超时
```
Error: timed out while waiting for target halted
```
**解决**:
1. 确认BOOT0=GND（Flash启动）
2. 检查3.3V供电是否正常
3. 尝试擦除芯片后重试：
   ```powershell
   openocd -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg -c "init; reset halt; stm32f1x mass_erase 0; exit"
   ```

### 5.3 调试中断点不生效
**解决**:
- 确保编译时使用Debug配置（`-O0 -g3`）
- 检查`.elf`文件是否包含调试信息

---

## 6. 串口调试与SWD调试对比

| 功能 | USART1串口 | CMSIS-DAP (SWD) |
|------|-----------|-----------------|
| 输出调试信息 | printf输出 | SWO或半主机模式 |
| 查看变量 | 需手动打印 | 实时监视窗口 |
| 设置断点 | ❌ | ✅ |
| 单步调试 | ❌ | ✅ |
| 寄存器查看 | ❌ | ✅ (SVD支持) |
| 外设状态 | ❌ | ✅ (SVD支持) |

**建议**: 
- 开发调试阶段使用 **CMSIS-DAP (SWD)**
- 生产环境日志使用 **USART2 (RS485)**

---

## 7. 当前项目配置总结

### 串口分配
- **USART1 (PA9/PA10)**: printf调试输出 → CMSIS-DAP虚拟串口或USB-TTL
- **USART2 (PA2/PA3)**: RS485通信 → CH340模块

### 调试方式
- **SWD调试**: PA13/PA14 → CMSIS-DAP (推荐)
- **串口调试**: PA9 → 查看printf输出

### 工作流程
1. 通过CMSIS-DAP下载程序并设置断点调试
2. 通过USART1查看printf调试信息（可选）
3. 通过USART2 + CH340测试RS485通信

---

## 8. 下一步操作

1. **验证OpenOCD安装**: 在终端运行 `openocd --version`
2. **连接CMSIS-DAP**: 确保硬件连接正确
3. **按F5启动调试**: VS Code会自动编译并下载程序
4. **设置断点测试**: 在main.c的while循环中设置断点

如有问题，请提供具体错误信息！
