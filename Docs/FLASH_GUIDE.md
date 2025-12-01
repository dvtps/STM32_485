# STM32固件烧录指南（分步骤详解）

## 🎯 目标
将编译好的固件 `build\Debug\STM32_485.elf` 烧录到STM32F103C8T6开发板

---

## 📦 准备工作

### 需要的硬件
- [ ] STM32F103C8T6开发板（正点原子M48Z-M3或兼容板）
- [ ] ST-Link V2调试器（或开发板自带的ST-Link）
- [ ] USB数据线
- [ ] （可选）RS485转USB模块（用于后续Modbus测试）

### 需要的软件
选择以下任一工具：

**选项A: STM32CubeProgrammer**（推荐，官方工具，图形界面）
- 下载地址: https://www.st.com/zh/development-tools/stm32cubeprog.html
- 优点: 官方支持，功能全面，有中文界面
- 缺点: 需要注册ST账号

**选项B: STM32 ST-LINK Utility**（老版工具，仍可用）
- 下载地址: https://www.st.com/zh/development-tools/stsw-link004.html
- 优点: 简单易用
- 缺点: 已停止更新

**选项C: Keil MDK**（如果已安装）
- 优点: 开发环境自带
- 缺点: 需要购买或破解

**选项D: OpenOCD**（命令行工具，高级用户）
- 优点: 开源免费，支持多种调试器
- 缺点: 命令行操作，配置复杂

---

## 🔌 方法1: 使用STM32CubeProgrammer烧录（推荐）

### 第1步: 连接硬件
```
ST-Link → STM32开发板
  GND   →   GND
  SWDIO →   SWDIO (PA13)
  SWCLK →   SWCLK (PA14)
  3.3V  →   3.3V (可选，用于供电)
```

### 第2步: 打开STM32CubeProgrammer
1. 启动软件
2. 右上角选择连接方式: **ST-LINK**
3. 点击刷新按钮，确认识别到ST-Link

### 第3步: 连接目标芯片
1. 点击 **Connect** 按钮
2. 如果连接成功，会显示芯片信息: **STM32F103C8**
3. 如果失败，检查硬件连接和驱动

### 第4步: 烧录固件
1. 点击左侧 **Erasing & Programming** 菜单
2. 在 **File path** 处，浏览选择:
   ```
   D:\STM32\Projects\ZDT\STM32_485\build\Debug\STM32_485.elf
   ```
3. 勾选以下选项:
   - ✅ **Verify programming** (验证烧录)
   - ✅ **Run after programming** (烧录后运行)
4. 点击 **Start Programming** 按钮

### 第5步: 验证烧录成功
- 显示 **File download complete**
- 开发板LED开始闪烁（如果代码中有LED控制）
- 点击 **Disconnect** 断开连接

---

## 🔌 方法2: 使用ST-LINK Utility烧录

### 第1步: 连接硬件（同方法1）

### 第2步: 打开ST-LINK Utility
1. 启动软件
2. 菜单栏: **Target → Connect** (或按F5)
3. 如果连接成功，窗口底部显示 **Connected via SWD**

### 第3步: 加载固件
1. 菜单栏: **File → Open File...**
2. 选择文件类型: **ELF Files (*.elf)**
3. 浏览选择:
   ```
   D:\STM32\Projects\ZDT\STM32_485\build\Debug\STM32_485.elf
   ```
4. 点击打开

### 第4步: 烧录
1. 菜单栏: **Target → Program & Verify...** (或按Ctrl+P)
2. 弹出对话框，确认设置:
   - Start address: **0x08000000**
   - ✅ Verify after programming
   - ✅ Reset after programming
3. 点击 **Start** 按钮

### 第5步: 验证
- 显示 **Verification...OK**
- 开发板复位并开始运行

---

## 🔌 方法3: 使用Keil MDK烧录

### 第1步: 在Keil中打开项目
（本项目使用CMake，不推荐此方法。如需使用Keil，需先导入项目）

### 第2步: 配置下载选项
1. 菜单: **Project → Options for Target**
2. 切换到 **Debug** 选项卡
3. 右侧下拉框选择: **ST-Link Debugger**
4. 点击 **Settings** 配置ST-Link
5. 切换到 **Flash Download** 选项卡
6. 确认 **Download Function** 已勾选
7. 点击 **OK** 保存

### 第3步: 下载固件
1. 菜单: **Flash → Download** (或按F8)
2. 等待下载完成

---

## 🔌 方法4: 使用OpenOCD烧录（命令行）

### 前置条件
- 已安装OpenOCD
- 已安装gcc-arm-none-eabi工具链

### 烧录命令
```bash
# 方式1: 自动烧录并复位
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program build/Debug/STM32_485.elf verify reset exit"

# 方式2: 交互式烧录
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg

# 在另一个终端执行
telnet localhost 4444
> reset halt
> flash write_image erase build/Debug/STM32_485.elf
> verify_image build/Debug/STM32_485.elf
> reset run
> exit
```

### PowerShell一键脚本
```powershell
# 保存为 flash.ps1
$elf_file = "build\Debug\STM32_485.elf"

if (!(Test-Path $elf_file)) {
    Write-Error "固件文件不存在: $elf_file"
    exit 1
}

Write-Host "正在烧录固件..." -ForegroundColor Green
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg `
  -c "program $elf_file verify reset exit"

if ($LASTEXITCODE -eq 0) {
    Write-Host "烧录成功！" -ForegroundColor Green
} else {
    Write-Host "烧录失败，请检查连接" -ForegroundColor Red
}
```

运行: `powershell .\flash.ps1`

---

## 🔧 常见问题排查

### 问题1: 无法连接到ST-Link
**可能原因**:
- ST-Link驱动未安装
- USB线接触不良
- ST-Link固件版本过旧

**解决方法**:
1. 安装ST-Link驱动（CubeProgrammer会自动安装）
2. 更换USB线或USB口
3. 使用CubeProgrammer升级ST-Link固件:
   - **Firmware upgrade** → **Open in update mode** → **Upgrade**

---

### 问题2: 连接成功但无法烧录
**可能原因**:
- 芯片被锁定（读保护）
- BOOT0引脚设置错误

**解决方法**:
1. 擦除芯片:
   - CubeProgrammer: **Erasing & Programming** → **Full chip erase**
2. 检查BOOT0引脚接GND（正常运行模式）

---

### 问题3: 烧录后程序不运行
**可能原因**:
- 复位未生效
- BOOT0/BOOT1配置错误
- 代码有死循环或硬件错误

**解决方法**:
1. 手动按复位按钮
2. 确认BOOT0接GND，BOOT1接GND
3. 通过ST-Link连接后查看PC寄存器地址（应为0x08000xxx）

---

### 问题4: LED不闪烁
**可能原因**:
- LED接线错误
- 代码中LED引脚配置错误

**验证方法**:
1. 通过USART1查看调试输出（PA9, 115200bps）
2. 如果能看到启动信息，说明程序正常运行

---

## ✅ 烧录成功标志

烧录成功后，你应该看到:

1. **软件提示**: "Programming success" 或 "Verification OK"
2. **LED指示**: 开发板LED闪烁（如果代码中有LED控制）
3. **串口输出**（通过USART1）:
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

---

## 📝 烧录记录

| 日期 | 固件版本 | 烧录工具 | 结果 | 备注 |
|------|---------|---------|------|------|
| 2025-12-01 | V3.0 | CubeProgrammer | ⏳ 待烧录 | 首次烧录 |

---

## 🚀 烧录后下一步

烧录成功后，按顺序进行:

1. ✅ 确认串口输出（USART1, 115200bps）
2. ✅ 连接RS485模块到USART2
3. ✅ 运行Python测试脚本（test_quick.py）
4. ✅ 验证Modbus通信
5. ✅ 连接电机测试运动控制

---

**版本**: V1.0  
**更新日期**: 2025-12-01  
**适用固件**: STM32_485 V3.0
