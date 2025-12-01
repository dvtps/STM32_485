# CMSIS-DAP调试配置完成指南

## ✅ 已成功配置

### 检测到的硬件
- **调试器**: ALIENTEK ATK-CMSIS-DAP
- **序列号**: ATK-20240820
- **目标芯片**: STM32F103C8

### 已安装工具
- ✅ Cortex-Debug扩展 (v1.12.1)
- ✅ PyOCD (v0.41.0)
- ✅ STM32F103C8支持包（正在下载）

---

## 🚀 开始调试

### 方法1: VS Code图形界面（推荐）

1. **连接硬件**:
   ```
   CMSIS-DAP → STM32F103C8
   ────────────────────────────
   SWDIO → PA13
   SWCLK → PA14  
   GND   → GND
   3.3V  → 3.3V (可选)
   ```

2. **按F5启动调试**
   - 或点击侧边栏"调试"图标
   - 选择 **"CMSIS-DAP Debug (PyOCD)"**
   - 程序会自动编译、下载、停在main函数

3. **调试操作**:
   - `F9`: 设置/取消断点
   - `F5`: 继续运行
   - `F10`: 单步跳过
   - `F11`: 单步进入
   - `Shift+F5`: 停止调试

### 方法2: 命令行烧录（不启动调试）

```powershell
# 使用PyOCD烧录
pyocd flash build/Debug/STM32_485.elf

# 使用STM32CubeProgrammer烧录
C:\ST\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=SWD -w build/Debug/STM32_485.elf -v -rst
```

---

## 🔧 PyOCD常用命令

```powershell
# 列出所有连接的调试器
pyocd list

# 查看芯片信息
pyocd info

# 烧录程序
pyocd flash build/Debug/STM32_485.elf

# 擦除芯片
pyocd erase -chip

# 复位芯片
pyocd reset

# 启动GDB Server（手动调试用）
pyocd gdbserver
```

---

## ⚠️ 常见问题

### 1. 找不到PyOCD
```powershell
# 将Python Scripts目录添加到PATH
# 路径通常为: C:\Users\你的用户名\AppData\Local\Programs\Python\Python3XX\Scripts
```

### 2. 无法检测到CMSIS-DAP
```
Error: No probe detected
```
**解决**:
- 重新插拔CMSIS-DAP
- 检查USB驱动是否正常
- 运行 `pyocd list` 验证

### 3. 连接超时
```
Error: Failed to connect to target
```
**解决**:
- 检查SWDIO/SWCLK连线
- 确认3.3V供电正常
- BOOT0跳线接GND
- 尝试擦除芯片: `pyocd erase -chip`

### 4. 下载速度慢
PyOCD首次使用时会下载芯片支持包，需要等待几分钟。

---

## 📊 当前项目配置

### 调试配置
- **配置文件**: `.vscode/launch.json`
- **调试器**: ALIENTEK ATK-CMSIS-DAP
- **GDB Server**: PyOCD
- **目标**: STM32F103C8

### 串口配置
- **USART1 (PA9/PA10)**: 调试输出, 115200, UTF-8
- **USART2 (PA2/PA3)**: RS485通信, 115200, UTF-8

### 测试程序
当前程序每秒通过USART2发送测试消息：
```
==============================
 USART2 RS485 Test #0
==============================
Time: 1000 ms
Status: Running OK
```

---

## 🎯 下一步

1. ✅ **按F5启动调试**
2. ✅ **在while循环设置断点**
3. ✅ **查看变量窗口**
4. ✅ **测试单步执行**
5. ✅ **测试USART2 RS485通信**

现在请按F5启动调试会话！
