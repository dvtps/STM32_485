# STM32 Modbus网关 - Python测试工具

## 快速开始

### 1. 安装依赖

```bash
pip install pymodbus
```

### 2. 修改串口号

编辑 `test_modbus_gateway.py`，修改第12行：

```python
SERIAL_PORT = 'COM3'  # 改成实际的串口号
```

### 3. 启用Modbus模式

编辑 `Core/App/main.c`，修改第33行：

```c
#define MODBUS_GATEWAY_MODE     1       /* 启用Modbus网关模式 */
```

编译并烧录到STM32。

### 4. 运行测试

```bash
python test_modbus_gateway.py
```

## 测试用例说明

### 测试1: 读取系统信息
- **功能码**: 0x03 (读保持寄存器)
- **读取内容**: 固件版本、硬件版本、设备ID
- **预期结果**: 
  - 固件版本: V1.0
  - 硬件版本: 0xF103 (STM32F103)
  - 设备ID: 1

### 测试2: 写单个寄存器
- **功能码**: 0x06 (写单个寄存器)
- **操作**: 设置电机1速度为1000 RPM
- **寄存器**: 0x0103 (MOTOR1_BASE + MOTOR_REG_SPEED)
- **预期结果**: 写入成功，回读验证一致

### 测试3: 写多个寄存器
- **功能码**: 0x10 (写多个寄存器)
- **操作**: 批量配置电机参数
  - 控制模式: 3 (梯形曲线)
  - 方向: 0 (顺时针)
  - 速度: 500 RPM
  - 加速度: 100
  - 位置: 360° (1圈)
- **预期结果**: 批量写入成功

### 测试4: 单电机位置控制
- **操作流程**:
  1. 使能电机 (写寄存器 ENABLE = 1)
  2. 确认参数已配置
  3. 执行位置运动命令 (写寄存器 EXEC_COMMAND = 3)
- **预期结果**: 电机转动1圈（360°），约5秒完成

### 测试5: 读取电机状态
- **功能码**: 0x04 (读输入寄存器)
- **读取内容**: 实时位置、速度、电流、电压、温度
- **寄存器**: 0x0500-0x0505 (MOTOR1_STATUS_BASE)
- **预期结果**: 返回电机实时状态（初期可能为0，需实现状态轮询）

### 测试6: 线圈操作
- **功能码**: 0x05 (写单个线圈), 0x01 (读线圈)
- **操作**: 快速使能/失能电机1
- **线圈地址**: 0x0C00 (COIL_MOTOR1_ENABLE)
- **预期结果**: 线圈状态切换成功

### 测试7: 紧急停止
- **功能码**: 0x05 (写单个线圈)
- **操作**: 触发全局急停线圈
- **线圈地址**: 0x0C10 (COIL_EMERGENCY_STOP)
- **预期结果**: 所有电机立即停止

## 故障排查

### 问题1: 无法连接串口
**错误信息**: `❌ 无法连接到串口`

**解决方法**:
1. 检查串口号是否正确（设备管理器查看）
2. 确认STM32已上电且CH340驱动已安装
3. 检查是否有其他程序占用串口（关闭串口助手）
4. 确认 `MODBUS_GATEWAY_MODE=1` 已编译烧录

### 问题2: 读取失败/超时
**错误信息**: `Modbus Error: [...]` 或 超时

**解决方法**:
1. 检查波特率是否匹配（STM32和Python都是115200）
2. 查看STM32串口是否工作（LED闪烁）
3. 使用串口助手发送原始Modbus帧验证（见下节）
4. 检查 `usart1_get_mode()` 是否返回 `USART1_MODE_MODBUS`

### 问题3: CRC校验错误
**原因**: Modbus帧CRC不匹配

**验证方法**:
```python
# 手动计算CRC16
from pymodbus.utilities import computeCRC
data = [0x01, 0x03, 0x00, 0x07, 0x00, 0x03]  # 读3个寄存器
crc = computeCRC(bytearray(data))
print(f"CRC: 0x{crc:04X}")  # 应匹配STM32计算结果
```

### 问题4: 电机无响应
**测试4执行后电机不动**

**排查步骤**:
1. 检查USART2与电机连接（RS485 A-B线）
2. 确认电机地址为1（与 `motor_id` 匹配）
3. 查看STM32调试输出（printf日志）
4. 测试原生 `motor_enable(1,1)` USMART命令

## 高级用法

### 1. 使用串口助手验证Modbus帧

**读取固件版本（寄存器0x0007）**:

发送（HEX）:
```
01 03 00 07 00 01 35 C8
```

解析:
- 01: 从机地址
- 03: 功能码（读保持寄存器）
- 00 07: 起始地址（0x0007）
- 00 01: 寄存器数量（1个）
- 35 C8: CRC16

预期响应:
```
01 03 02 01 00 XX XX
```
- 01 03: 地址+功能码
- 02: 字节数（2字节）
- 01 00: 数据（0x0100 = V1.0）
- XX XX: CRC16

### 2. 修改测试脚本

**添加新测试用例**:

```python
def test_08_multi_motor_sync(client):
    """测试8: 多电机同步运动"""
    print("\n" + "="*60)
    print("测试8: 多电机同步运动（电机1和2同时启动）")
    print("="*60)
    
    try:
        # 配置电机1和2的同步标志
        client.write_register(MOTOR1_BASE + MOTOR_REG_SYNC_FLAG, 1, slave=SLAVE_ADDRESS)
        client.write_register(0x0140 + MOTOR_REG_SYNC_FLAG, 1, slave=SLAVE_ADDRESS)
        
        # 发送位置命令（不立即执行）
        client.write_register(MOTOR1_BASE + MOTOR_REG_EXEC_COMMAND, CMD_POS_MOVE, slave=SLAVE_ADDRESS)
        client.write_register(0x0140 + MOTOR_REG_EXEC_COMMAND, CMD_POS_MOVE, slave=SLAVE_ADDRESS)
        
        # 触发同步运动
        client.write_coil(0x0C14, True, slave=SLAVE_ADDRESS)  # SYNC_MOTION_TRIGGER
        
        print_success("同步运动已触发")
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False
```

将函数添加到 `tests` 列表即可。

### 3. 批量读写示例

**批量读取电机1-4的速度**:

```python
# 电机1-4连续，每个64寄存器，速度偏移+3
speeds = []
for motor_id in range(1, 5):
    base = 0x0100 + (motor_id - 1) * 64
    result = client.read_holding_registers(base + 3, 1, slave=SLAVE_ADDRESS)
    if not result.isError():
        speeds.append(result.registers[0] / 10.0)  # 转换为RPM

print(f"电机1-4速度: {speeds}")
```

## 性能测试

### 1. 响应时间测试

```python
import time

start = time.time()
for _ in range(100):
    client.read_holding_registers(REG_SYS_FIRMWARE_VERSION, 1, slave=SLAVE_ADDRESS)
end = time.time()

avg_time = (end - start) / 100 * 1000  # ms
print(f"平均响应时间: {avg_time:.2f} ms")
```

**目标性能**: < 10ms

### 2. 吞吐量测试

```python
# 批量读取256个寄存器（最大限制）
start = time.time()
result = client.read_holding_registers(0x0100, 125, slave=SLAVE_ADDRESS)  # Modbus最大125个
end = time.time()

if not result.isError():
    print(f"读取125个寄存器耗时: {(end-start)*1000:.2f} ms")
    print(f"吞吐量: {250 / (end-start)} 字节/秒")  # 125寄存器 = 250字节
```

## 参考资料

- **Modbus协议**: `Docs/Modbus寄存器映射表V2.1-完整版.md`
- **寄存器地址**: 章节2-7完整定义
- **功能码映射**: 章节9-10
- **错误码**: 章节8

## 联系方式

问题反馈: [GitHub Issues](#)
技术支持: [项目文档](../README.md)
