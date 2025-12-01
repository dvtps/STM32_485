# STM32固件烧录 - pyocd版本

## 🚀 快速烧录

### 方法1: 一键烧录（推荐）
```cmd
双击运行: flash.bat
```

### 方法2: 命令行烧录
```powershell
# 基础烧录（自动检测目标芯片）
python -m pyocd load build\Debug\STM32_485.elf

# 指定目标芯片
python -m pyocd load -t stm32f103c8 build\Debug\STM32_485.elf

# 全片擦除后烧录
python -m pyocd load -t stm32f103c8 -e chip build\Debug\STM32_485.elf
```

---

## 📦 安装pyocd

### 检查是否已安装
```powershell
python -m pyocd --version
```

### 安装pyocd
```powershell
# 标准安装
pip install pyocd

# 或使用国内镜像加速
pip install pyocd -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 安装芯片支持包（首次使用）
```powershell
# 安装STM32F1系列支持
python -m pyocd pack install stm32f103c8
```

---

## 🔌 硬件连接

### ST-Link连接方式
```
ST-Link → STM32F103C8
  GND   →   GND
  SWDIO →   SWDIO (PA13)
  SWCLK →   SWCLK (PA14)
  3.3V  →   3.3V (可选供电)
```

---

## 🛠️ pyocd常用命令

### 1. 列出已连接的调试器
```powershell
python -m pyocd list
```

**预期输出示例**:
```
  #   Probe                    Unique ID
--------------------------------------------------------
  0   STMicroelectronics...    066CFF565551717867205727
```

### 2. 查看支持的目标芯片
```powershell
python -m pyocd list --targets
```

### 3. 擦除芯片
```powershell
# 全片擦除
python -m pyocd erase -t stm32f103c8 --chip

# 擦除指定扇区
python -m pyocd erase -t stm32f103c8 --sector 0
```

### 4. 读取芯片信息
```powershell
python -m pyocd info
```

### 5. GDB调试服务器
```powershell
# 启动GDB服务器（端口3333）
python -m pyocd gdbserver -t stm32f103c8

# 在另一个终端连接
arm-none-eabi-gdb build\Debug\STM32_485.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) continue
```

---

## 🐛 常见问题排查

### 问题1: 找不到调试器
**现象**:
```
No available probe found
```

**解决方法**:
1. 检查ST-Link USB连接
2. 确认ST-Link驱动已安装
3. 尝试更换USB口
4. Windows: 打开设备管理器查看是否有"STMicroelectronics STLink"

### 问题2: 无法连接目标芯片
**现象**:
```
Error: Unable to connect to target
```

**解决方法**:
1. 检查SWDIO/SWCLK连接
2. 确认开发板供电正常
3. 尝试降低SWD频率:
   ```powershell
   python -m pyocd flash -t stm32f103c8 -f 1000000 build\Debug\STM32_485.elf
   ```

### 问题3: 芯片读保护
**现象**:
```
Error: Flash is locked
```

**解决方法**:
```powershell
# 解除读保护（会擦除所有数据）
python -m pyocd erase -t stm32f103c8 --chip --mass-erase
```

### 问题4: 目标芯片不支持
**现象**:
```
Target type 'stm32f103c8' not recognized
```

**解决方法**:
```powershell
# 安装芯片支持包
python -m pyocd pack install stm32f103c8

# 或安装完整STM32F1系列
python -m pyocd pack install stm32f1
```

---

## 📊 烧录验证

### 方法1: 查看调试输出
连接USART1到USB-TTL模块:
- 波特率: 115200
- 数据位: 8
- 停止位: 1
- 校验位: None

**预期输出**:
```
============================================================
 STM32 步进电机控制系统 V3.0
 基于 Emm_V5 协议
 编译日期: Dec  1 2025
============================================================

[INIT] 系统初始化完成
[INIT] 电机通信初始化完成
[INIT] Modbus RTU initialized: Address=1, Baudrate=115200
```

### 方法2: LED指示
- LED0应该闪烁（如果代码中有LED控制）

### 方法3: 读取Flash内容
```powershell
# 读取前256字节（中断向量表）
python -m pyocd read -t stm32f103c8 0x08000000 256 -o hex
```

---

## 🔄 重新烧录流程

如果需要修改代码后重新烧录:

```powershell
# 1. 重新编译
cmake --build --preset Debug

# 2. 烧录新固件
python -m pyocd flash -t stm32f103c8 build\Debug\STM32_485.elf

# 3. 复位运行（或按开发板复位键）
python -m pyocd reset -t stm32f103c8
```

---

## 🎯 一键脚本

项目中已包含 `flash.bat`，双击即可使用。

**脚本功能**:
- ✅ 自动检查固件是否存在
- ✅ 自动检查pyocd是否安装
- ✅ 显示已连接的调试器
- ✅ 确认后执行烧录
- ✅ 显示烧录结果和下一步操作

---

## 📝 PowerShell高级脚本

保存为 `flash.ps1`:

```powershell
# STM32固件烧录脚本 (pyocd)
param(
    [string]$Firmware = "build\Debug\STM32_485.elf",
    [string]$Target = "stm32f103c8",
    [switch]$Erase,
    [switch]$Verify
)

# 检查固件
if (!(Test-Path $Firmware)) {
    Write-Error "固件不存在: $Firmware"
    exit 1
}

Write-Host "`n[固件信息]" -ForegroundColor Cyan
Get-Item $Firmware | Format-List Name, Length, LastWriteTime

# 检查pyocd
Write-Host "[检查pyocd]" -ForegroundColor Cyan
try {
    $version = python -m pyocd --version 2>&1
    Write-Host $version -ForegroundColor Green
} catch {
    Write-Error "pyocd未安装，请运行: pip install pyocd"
    exit 1
}

# 列出调试器
Write-Host "`n[扫描调试器]" -ForegroundColor Cyan
python -m pyocd list

# 构建命令
$cmd = "python -m pyocd flash -t $Target"
if ($Erase) { $cmd += " --erase chip" }
if ($Verify) { $cmd += " --verify" }
$cmd += " $Firmware"

# 确认
Write-Host "`n准备执行: " -NoNewline -ForegroundColor Yellow
Write-Host $cmd -ForegroundColor White
$confirm = Read-Host "继续？(y/n)"
if ($confirm -ne 'y') {
    Write-Host "已取消" -ForegroundColor Yellow
    exit 0
}

# 烧录
Write-Host "`n[开始烧录]" -ForegroundColor Green
Invoke-Expression $cmd

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n✅ 烧录成功！" -ForegroundColor Green
} else {
    Write-Host "`n❌ 烧录失败" -ForegroundColor Red
    exit 1
}
```

**使用方法**:
```powershell
# 基础烧录
.\flash.ps1

# 擦除后烧录
.\flash.ps1 -Erase

# 烧录后验证
.\flash.ps1 -Verify

# 指定固件文件
.\flash.ps1 -Firmware "path\to\firmware.elf"
```

---

## 🚀 下一步

烧录成功后:

1. **验证运行**:
   ```powershell
   # 查看USART1调试输出 (115200bps)
   # 应该看到启动信息和Modbus初始化信息
   ```

2. **连接RS485**:
   - USART2 (PA2/PA3) → RS485模块 → PC

3. **运行测试**:
   ```powershell
   cd Docs
   python test_quick.py
   ```

---

**版本**: V1.0  
**更新日期**: 2025-12-01  
**适用工具**: pyocd
