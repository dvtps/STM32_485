# Python Modbus RTU 测试快速启动指南

## 📦 安装依赖

```bash
# 安装 pymodbus 库
pip install pymodbus

# 或使用国内镜像加速
pip install pymodbus -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 🎯 测试脚本说明

### 1. `test_modbus_simple.py` - 简化版测试
**适用场景**: 快速验证Modbus通信是否正常

**功能**:
- ✅ 读取固件版本（地址0x0007）
- ✅ 读取硬件版本（地址0x0008）
- ✅ 写入电机速度寄存器（地址0x0103）
- ✅ 写入线圈使能电机（地址0x0C00）

**使用方法**:
```bash
# 修改串口号（第14行）
SERIAL_PORT = 'COM3'  # Windows下改为实际COM口，Linux下改为/dev/ttyUSB0

# 运行测试
python test_modbus_simple.py
```

**预期输出**:
```
============================================================
 STM32 Modbus网关简化测试 (pymodbus 3.x)
============================================================
串口: COM3, 波特率: 115200, 从机地址: 1
============================================================

✅ 已连接到 COM3

【测试1】读取固件版本（地址0x0007）
✅ 固件版本: 0x0300 (V3.0)

【测试2】读取硬件版本（地址0x0008）
✅ 硬件版本: 0x0001

【测试3】写入电机1速度寄存器（地址0x0103, 值10000）
✅ 写入成功
✅ 回读验证: 10000 (应为10000)
```

---

### 2. `test_modbus_gateway.py` - 完整版测试套件
**适用场景**: 全面测试电机控制功能

**测试用例**:
1. ✅ **读取系统信息** - 固件版本、硬件版本、设备ID
2. ✅ **写单个寄存器** - 设置电机速度为1000 RPM
3. ✅ **写多个寄存器** - 批量配置运动参数
4. ✅ **单电机位置控制** - 执行1圈运动
5. ✅ **读取电机状态** - 实时位置、速度、电流
6. ✅ **线圈操作** - 快速使能/失能
7. ✅ **紧急停止** - 触发全局急停

**使用方法**:
```bash
# 修改串口号（第13行）
SERIAL_PORT = 'COM7'  # 改为实际COM口

# 运行完整测试
python test_modbus_gateway.py
```

**预期输出**:
```
============================================================
 STM32 Modbus网关测试程序 V1.0
============================================================
串口: COM7
波特率: 115200
从机地址: 1
超时时间: 1.0s
============================================================
✅ 已连接到 COM7

>>> 运行测试 1/7: 读取系统信息
============================================================
测试1: 读取系统信息（固件版本、硬件版本、设备ID）
============================================================
✅ 读取成功:
  - 固件版本: V3.0
  - 硬件版本: 0x0001
  - 设备ID: 1

>>> 运行测试 2/7: 写单个寄存器
... (后续测试)

============================================================
 测试结果汇总
============================================================
总计: 7 个测试
✅ 通过: 7
❌ 失败: 0
============================================================
```

---

## 🔧 配置参数说明

### 串口配置（两个脚本通用）
```python
SERIAL_PORT = 'COM7'        # Windows: COM1-COM255, Linux: /dev/ttyUSB0
BAUDRATE = 115200           # 必须与STM32配置一致（app_config.h）
SLAVE_ADDRESS = 1           # 从机地址（MODBUS_SLAVE_ADDRESS）
TIMEOUT = 1.0               # 超时时间（秒），RS485建议1-2秒
```

### 常见串口号识别
- **Windows**: 
  - 打开设备管理器 → 端口(COM和LPT) → 查看"USB-SERIAL CH340 (COMx)"
  
- **Linux**:
  ```bash
  # 查看所有串口设备
  ls /dev/ttyUSB* /dev/ttyACM*
  
  # 给予串口权限
  sudo chmod 666 /dev/ttyUSB0
  ```

- **macOS**:
  ```bash
  ls /dev/cu.usbserial-*
  ```

---

## 🐛 常见问题排查

### 问题1: ModuleNotFoundError: No module named 'pymodbus'
**原因**: 未安装pymodbus库

**解决**:
```bash
pip install pymodbus
```

---

### 问题2: ❌ 无法连接到串口
**可能原因**:
1. 串口号错误（COM3不存在）
2. 串口被其他程序占用（串口助手/PuTTY）
3. USB线接触不良
4. STM32未上电

**排查步骤**:
```python
# 1. 列出所有可用串口
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port in ports:
    print(port)

# 2. 检查串口权限（Linux）
sudo chmod 666 /dev/ttyUSB0

# 3. 确认STM32是否运行
# 通过USART1调试串口查看printf输出
```

---

### 问题3: 读取超时或无响应
**可能原因**:
1. FEATURE_MODBUS_ENABLE=0（未启用Modbus）
2. RS485 A/B线接反
3. 波特率不匹配
4. STM32固件未烧录或卡死

**解决方案**:
```c
// 1. 检查 Core/App/app_config.h
#define FEATURE_MODBUS_ENABLE      1  // 确保为1

// 2. 检查RS485接线
// A-A, B-B（不是A-B交叉）

// 3. 检查波特率
#define MODBUS_BAUDRATE           115200  // 必须与Python一致

// 4. 查看USART1调试日志
// 应看到: "Modbus RTU initialized: Address=1, Baudrate=115200"
```

---

### 问题4: CRC校验失败
**可能原因**:
- 数据传输错误
- 电磁干扰
- RS485总线终端电阻未配置

**解决方案**:
```python
# 1. 降低波特率测试
BAUDRATE = 9600  # 从115200降低到9600

# 2. 添加调试日志
import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# 3. 检查接线质量
# 使用示波器查看RS485波形
```

---

### 问题5: 写入成功但电机不动
**排查步骤**:

```python
# 步骤1: 确认Modbus通信正常（测试1通过）
python test_modbus_gateway.py  # 测试1应通过

# 步骤2: 手动使能电机
result = client.write_register(0x0100, 1, slave=1)  # ENABLE=1
print(result)

# 步骤3: 触发执行命令
result = client.write_register(0x0108, 0x0003, slave=1)  # CMD_POS_MOVE
print(result)

# 步骤4: 检查STM32调试日志
# USART1应输出: "[EMM_V5] Motor pos control: ..."
```

---

## 📊 测试流程建议

### 🚀 初次测试（推荐流程）

**第1步**: 硬件连接
```
STM32 USART2 (RS485) → USB-RS485转换器 → PC
STM32 USART1 (调试)   → USB-TTL模块 → 串口助手
```

**第2步**: 打开2个终端/命令提示符窗口
- 窗口1: 运行Python测试脚本
- 窗口2: 串口助手监听USART1调试输出（115200bps）

**第3步**: 运行简化测试
```bash
cd D:\STM32\Projects\ZDT\STM32_485\Docs
python test_modbus_simple.py
```

**第4步**: 如果简化测试通过，运行完整测试
```bash
python test_modbus_gateway.py
```

**第5步**: 连接实际电机，执行电机控制测试（测试4）

---

### 🎯 快速验证脚本（复制即用）

创建 `quick_test.py`:
```python
#!/usr/bin/env python3
from pymodbus.client import ModbusSerialClient

# ========== 修改这里 ==========
SERIAL_PORT = 'COM7'  # 改为你的COM口
# ==============================

client = ModbusSerialClient(port=SERIAL_PORT, baudrate=115200, timeout=1)

if client.connect():
    print("✅ 连接成功")
    
    # 读取固件版本
    result = client.read_holding_registers(0x0007, 1, slave=1)
    if not result.isError():
        print(f"✅ 固件版本: V{result.registers[0]>>8}.{result.registers[0]&0xFF}")
    else:
        print(f"❌ 读取失败: {result}")
    
    client.close()
else:
    print("❌ 连接失败，请检查串口号")
```

运行:
```bash
python quick_test.py
```

---

## 📚 扩展测试示例

### 示例1: 连续读取电机状态（实时监控）
```python
import time
from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(port='COM7', baudrate=115200, timeout=1)
client.connect()

try:
    while True:
        # 读取电机1状态寄存器
        result = client.read_input_registers(0x0500, 6, slave=1)
        if not result.isError():
            pos_h, pos_l, speed, current, voltage, temp = result.registers
            position = (pos_h << 16) | pos_l
            print(f"位置:{position:6d}  速度:{speed:4d}  电流:{current:4d}mA  "
                  f"电压:{voltage:5d}mV  温度:{temp:2d}℃")
        time.sleep(0.5)  # 每500ms读取一次
except KeyboardInterrupt:
    print("\n停止监控")
finally:
    client.close()
```

### 示例2: 批量控制多个电机
```python
from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(port='COM7', baudrate=115200, timeout=1)
client.connect()

# 电机1-3基地址
motors = [0x0100, 0x0140, 0x0180]

# 批量使能
for base in motors:
    client.write_register(base + 0, 1, slave=1)  # ENABLE=1
    print(f"电机 {(base-0x0100)//0x40 + 1} 已使能")

# 批量触发运动
for base in motors:
    client.write_register(base + 8, 3, slave=1)  # EXEC_COMMAND=CMD_POS_MOVE
    print(f"电机 {(base-0x0100)//0x40 + 1} 开始运动")

client.close()
```

---

## 🎓 学习资源

- **pymodbus官方文档**: https://pymodbus.readthedocs.io/
- **Modbus协议标准**: Modbus_Application_Protocol_V1_1b3.pdf
- **本项目文档**: 
  - `Docs/doc_Y57/02-通信协议.md` - Emm_V5协议
  - `Docs/Modbus_RTU_Testing_Guide.md` - 完整测试指南
  - `Drivers/Middlewares/MODBUS/modbus_gateway.h` - 寄存器地址映射

---

**版本**: V1.0  
**更新日期**: 2025-12-01  
**作者**: STM32_485 Project Team
