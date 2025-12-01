# Modbus寄存器映射表设计 V2.0 (企业级架构)

## 设计目标

构建**工业级Modbus网关**，支持复杂应用场景：
- 上位机通过Modbus控制STM32网关
- STM32翻译为Emm_V5命令控制多台电机
- 支持**16台电机**全功能控制
- 支持**高级参数配置**（PID、细分、保护参数）
- 支持**多段轨迹规划**（最多64段缓冲）
- 支持**间接寻址**扩展无限寄存器

## V2.0 架构升级

### 关键改进
1. **寄存器扩展**：每电机从16个→32个寄存器
2. **间接寻址**：通过INDEX机制访问扩展参数
3. **轨迹缓冲**：支持预加载64段运动指令
4. **事件机制**：中断触发、到位通知、故障告警
5. **多电机**：支持1-16台（可扩展到32台）

---

## 1. Modbus通信参数

```
从机地址：   0x01 (STM32网关地址，固定)
波特率：     115200 (与电机通信一致)
数据位：     8
停止位：     1
校验位：     无
CRC校验：    Modbus CRC16 (低字节在前)
```

---

## 2. 寄存器地址分配总览 (V2.0分层架构)

```
地址范围          类型          功能区域                        访问权限
═════════════════════════════════════════════════════════════════════════
【全局控制区】
0x0000-0x001F    Holding Reg   系统全局配置                    R/W
0x0020-0x003F    Holding Reg   间接寻址控制器                  R/W
0x0040-0x005F    Holding Reg   轨迹缓冲区控制                  R/W
0x0060-0x007F    Input Reg     系统状态反馈                    R

【电机直接访问区】(每电机32个寄存器)
0x0100-0x011F    Holding Reg   1号电机基础控制                 R/W
0x0120-0x013F    Holding Reg   2号电机基础控制                 R/W
0x0140-0x015F    Holding Reg   3号电机基础控制                 R/W
0x0160-0x017F    Holding Reg   4号电机基础控制                 R/W
0x0180-0x019F    Holding Reg   5号电机基础控制                 R/W
0x01A0-0x01BF    Holding Reg   6号电机基础控制                 R/W
0x01C0-0x01DF    Holding Reg   7号电机基础控制                 R/W
0x01E0-0x01FF    Holding Reg   8号电机基础控制                 R/W

0x0300-0x031F    Input Reg     1号电机状态反馈                 R
0x0320-0x033F    Input Reg     2号电机状态反馈                 R
0x0340-0x035F    Input Reg     3号电机状态反馈                 R
0x0360-0x037F    Input Reg     4号电机状态反馈                 R
0x0380-0x039F    Input Reg     5号电机状态反馈                 R
0x03A0-0x03BF    Input Reg     6号电机状态反馈                 R
0x03C0-0x03DF    Input Reg     7号电机状态反馈                 R
0x03E0-0x03FF    Input Reg     8号电机状态反馈                 R

【间接访问区】(扩展参数)
0x0500-0x051F    Holding Reg   间接参数缓冲区 (32个通用寄存器) R/W
0x0520-0x053F    Input Reg     间接状态缓冲区 (32个反馈寄存器) R

【轨迹缓冲区】
0x0600-0x067F    Holding Reg   轨迹段缓冲 (64段×2寄存器)       R/W

【线圈和离散量】
0x0800-0x081F    Coils         快速开关控制 (32个布尔量)       R/W
0x0900-0x091F    Discrete In   快速状态反馈 (32个布尔量)       R
```

**寻址公式 (V2.0)**：
```c
/* 电机控制寄存器基址 = 0x0100 + (motor_id - 1) * 0x20 */
#define MOTOR_CTRL_BASE(id)   (0x0100 + ((id) - 1) * 0x20)
#define MOTOR_STATUS_BASE(id) (0x0300 + ((id) - 1) * 0x20)

/* 例如：
 * 1号电机：0x0100-0x011F (32个寄存器)
 * 2号电机：0x0120-0x013F
 * 8号电机：0x01E0-0x01FF
 */
```

---

## 3. 控制寄存器详细定义 (Holding Registers)

### 3.1 基础控制寄存器 (每个电机16个寄存器)

**以1号电机为例（0x0000-0x000F）**：

| 地址 | 名称 | R/W | 范围 | 说明 | 对应API |
|------|------|-----|------|------|---------|
| **0x0000** | MOTOR_ENABLE | R/W | 0/1 | 电机使能<br>0=失能, 1=使能 | `Emm_V5_En_Control()` |
| **0x0001** | MOTION_MODE | R/W | 0-2 | 运动模式<br>0=停止, 1=位置, 2=速度 | 模式切换逻辑 |
| **0x0002** | DIRECTION | R/W | 0/1 | 运动方向<br>0=CW顺时针, 1=CCW逆时针 | 参数传递 |
| **0x0003** | TARGET_SPEED | R/W | 0-5000 | 目标速度(RPM) | `Emm_V5_Pos/Vel_Control()` |
| **0x0004** | ACCELERATION | R/W | 0-255 | 加速度<br>0=直接启动 | 参数传递 |
| **0x0005** | TARGET_POS_H | R/W | 0-65535 | 目标位置高16位<br>完整32位脉冲数 | `Emm_V5_Pos_Control()` |
| **0x0006** | TARGET_POS_L | R/W | 0-65535 | 目标位置低16位 | (组合成uint32_t) |
| **0x0007** | MOTION_TYPE | R/W | 0/1 | 位置模式类型<br>0=相对, 1=绝对 | raF参数 |
| **0x0008** | EXEC_COMMAND | W | 0-10 | 执行命令<br>(见3.2命令表) | 触发API调用 |
| **0x0009** | HOMING_MODE | R/W | 0-3 | 回零模式<br>0=就近, 1=方向, 2=碰撞, 3=限位 | `Emm_V5_Origin_*()` |
| **0x000A** | HOMING_SPEED | R/W | 0-5000 | 回零速度(RPM) | 回零参数 |
| **0x000B** | HOMING_TIMEOUT_H | R/W | 0-65535 | 回零超时高16位(ms) | o_tm参数 |
| **0x000C** | HOMING_TIMEOUT_L | R/W | 0-65535 | 回零超时低16位(ms) | (组合成uint32_t) |
| **0x000D** | STALL_CURRENT | R/W | 0-5000 | 堵转检测电流(mA) | sl_ma参数 |
| **0x000E** | STALL_TIME | R/W | 0-65535 | 堵转检测时间(ms) | sl_ms参数 |
| **0x000F** | CONTROL_MODE | R/W | 1-2 | 控制模式<br>1=开环, 2=闭环 | `Emm_V5_Modify_Ctrl_Mode()` |

### 3.2 执行命令码表 (0x0008寄存器)

| 命令值 | 命令名称 | 说明 | 对应API |
|-------|---------|------|---------|
| **0** | CMD_NONE | 无操作（默认值） | - |
| **1** | CMD_ENABLE | 使能电机 | `Emm_V5_En_Control(addr, true, false)` |
| **2** | CMD_DISABLE | 失能电机 | `Emm_V5_En_Control(addr, false, false)` |
| **3** | CMD_POS_MOVE | 位置运动 | `Emm_V5_Pos_Control(...)` |
| **4** | CMD_VEL_MOVE | 速度运动 | `Emm_V5_Vel_Control(...)` |
| **5** | CMD_STOP | 立即停止 | `Emm_V5_Stop_Now(...)` |
| **6** | CMD_HOMING | 触发回零 | `Emm_V5_Origin_Trigger_Return(...)` |
| **7** | CMD_RESET_POS | 位置清零 | `Emm_V5_Reset_CurPos_To_Zero(...)` |
| **8** | CMD_RESET_CLOG | 解除堵转 | `Emm_V5_Reset_Clog_Pro(...)` |
| **9** | CMD_SYNC_MOTION | 同步运动触发 | `Emm_V5_Synchronous_motion(0)` |
| **10** | CMD_SAVE_PARAMS | 保存参数到Flash | 内部EEPROM操作 |

**执行逻辑**：
```c
/* 主机写入命令值到0x0008后，STM32立即执行对应API */
if (holding_reg[0x0008] == CMD_POS_MOVE) {
    uint32_t pos = (holding_reg[0x0005] << 16) | holding_reg[0x0006];
    Emm_V5_Pos_Control(
        motor_addr,
        holding_reg[0x0002],  // direction
        holding_reg[0x0003],  // speed
        holding_reg[0x0004],  // acc
        pos,
        holding_reg[0x0007],  // raF
        false
    );
    holding_reg[0x0008] = 0;  // 命令执行后清零
}
```

---

## 4. 状态寄存器详细定义 (Input Registers)

### 4.1 电机状态反馈 (只读)

**以1号电机为例（0x0100-0x010F）**：

| 地址 | 名称 | 范围 | 说明 | 对应API |
|------|------|------|------|---------|
| **0x0100** | MOTOR_STATUS | 0-3 | 电机状态<br>0=停止, 1=运行, 2=堵转, 3=回零中 | `Emm_V5_Read_Sys_Params(S_FLAG)` |
| **0x0101** | CURRENT_POS_H | 0-65535 | 当前位置高16位(脉冲) | `Emm_V5_Read_Sys_Params(S_CPOS)` |
| **0x0102** | CURRENT_POS_L | 0-65535 | 当前位置低16位 | (解析成int32_t) |
| **0x0103** | CURRENT_SPEED | 0-5000 | 当前速度(RPM) | `Emm_V5_Read_Sys_Params(S_VEL)` |
| **0x0104** | POS_ERROR_H | 0-65535 | 位置误差高16位 | `Emm_V5_Read_Sys_Params(S_PERR)` |
| **0x0105** | POS_ERROR_L | 0-65535 | 位置误差低16位 | (解析成int16_t) |
| **0x0106** | BUS_VOLTAGE | 0-65535 | 总线电压(mV) | `Emm_V5_Read_Sys_Params(S_VBUS)` |
| **0x0107** | ENCODER_VALUE | 0-65535 | 编码器值 | `Emm_V5_Read_Sys_Params(S_ENCL)` |
| **0x0108** | HOMING_STATUS | 0-2 | 回零状态<br>0=未回零, 1=回零中, 2=已完成 | `Emm_V5_Read_Sys_Params(S_ORG)` |
| **0x0109** | ERROR_CODE | 0-255 | 错误代码<br>0=正常, 1=堵转, 2=超时... | 状态解析 |
| **0x010A** | FIRMWARE_VER | 0-65535 | 固件版本号 | `Emm_V5_Read_Sys_Params(S_VER)` |
| **0x010B** | RESERVED_1 | - | 保留 | - |
| **0x010C** | RESERVED_2 | - | 保留 | - |
| **0x010D** | RESERVED_3 | - | 保留 | - |
| **0x010E** | RESERVED_4 | - | 保留 | - |
| **0x010F** | RESERVED_5 | - | 保留 | - |

---

## 5. 线圈寄存器 (Coils - 布尔量)

**单比特读写控制**：

| 地址 | 名称 | R/W | 说明 |
|------|------|-----|------|
| **0x0200** | MOTOR1_ENABLE | R/W | 1号电机使能开关 |
| **0x0201** | MOTOR2_ENABLE | R/W | 2号电机使能开关 |
| **0x0202** | MOTOR3_ENABLE | R/W | 3号电机使能开关 |
| **0x0203** | MOTOR4_ENABLE | R/W | 4号电机使能开关 |
| **0x0204** | MOTOR5_ENABLE | R/W | 5号电机使能开关 |
| **0x0205** | MOTOR6_ENABLE | R/W | 6号电机使能开关 |
| **0x0206** | MOTOR7_ENABLE | R/W | 7号电机使能开关 |
| **0x0207** | MOTOR8_ENABLE | R/W | 8号电机使能开关 |
| **0x0208** | GLOBAL_ENABLE | R/W | 全局使能（广播地址0） |
| **0x0209** | EMERGENCY_STOP | W | 紧急停止所有电机 |

---

## 6. 离散输入寄存器 (Discrete Inputs - 只读布尔量)

**单比特状态反馈**：

| 地址 | 名称 | 说明 |
|------|------|------|
| **0x0300** | MOTOR1_RUNNING | 1号电机运行中 |
| **0x0301** | MOTOR2_RUNNING | 2号电机运行中 |
| **0x0302** | MOTOR3_RUNNING | 3号电机运行中 |
| **0x0303** | MOTOR4_RUNNING | 4号电机运行中 |
| **0x0304** | MOTOR5_RUNNING | 5号电机运行中 |
| **0x0305** | MOTOR6_RUNNING | 6号电机运行中 |
| **0x0306** | MOTOR7_RUNNING | 7号电机运行中 |
| **0x0307** | MOTOR8_RUNNING | 8号电机运行中 |
| **0x0308** | MOTOR1_AT_POS | 1号电机到位标志 |
| **0x0309** | MOTOR2_AT_POS | 2号电机到位标志 |
| **0x030A** | MOTOR3_AT_POS | 3号电机到位标志 |
| **0x030B** | MOTOR4_AT_POS | 4号电机到位标志 |
| **0x030C** | MOTOR5_AT_POS | 5号电机到位标志 |
| **0x030D** | MOTOR6_AT_POS | 6号电机到位标志 |
| **0x030E** | MOTOR7_AT_POS | 7号电机到位标志 |
| **0x030F** | MOTOR8_AT_POS | 8号电机到位标志 |

---

## 7. Modbus功能码支持

| 功能码 | 名称 | 说明 | 支持 |
|-------|------|------|------|
| **0x01** | Read Coils | 读线圈状态 | ✅ |
| **0x02** | Read Discrete Inputs | 读离散输入 | ✅ |
| **0x03** | Read Holding Registers | 读保持寄存器 | ✅ |
| **0x04** | Read Input Registers | 读输入寄存器 | ✅ |
| **0x05** | Write Single Coil | 写单个线圈 | ✅ |
| **0x06** | Write Single Register | 写单个寄存器 | ✅ |
| **0x0F** | Write Multiple Coils | 写多个线圈 | ✅ |
| **0x10** | Write Multiple Registers | 写多个寄存器 | ✅ |

---

## 8. 典型应用示例

### 8.1 使能1号电机并转动1圈

```python
# Python pymodbus示例
from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(port='COM3', baudrate=115200)

# 写控制参数到1号电机寄存器（基址0x0000）
client.write_register(0x0002, 0)      # 方向=CW
client.write_register(0x0003, 1000)   # 速度=1000RPM
client.write_register(0x0004, 20)     # 加速度=20
client.write_register(0x0005, 0)      # 位置高字节=0
client.write_register(0x0006, 3200)   # 位置低字节=3200 (1圈)
client.write_register(0x0007, 0)      # 相对运动

# 执行命令
client.write_register(0x0008, 1)      # CMD_ENABLE (使能)
time.sleep(0.1)
client.write_register(0x0008, 3)      # CMD_POS_MOVE (位置运动)

# 读取状态
status = client.read_input_registers(0x0100, 3)
print(f"电机状态: {status.registers[0]}")
print(f"当前位置: {(status.registers[1]<<16) | status.registers[2]}")
```

### 8.2 批量写入参数（0x10功能码）

```python
# 一次性写入多个参数（更高效）
values = [
    0,       # 0x0002: 方向=CW
    1500,    # 0x0003: 速度=1500RPM
    30,      # 0x0004: 加速度=30
    0,       # 0x0005: 位置高=0
    6400,    # 0x0006: 位置低=6400 (2圈)
    0        # 0x0007: 相对运动
]
client.write_registers(0x0002, values)
client.write_register(0x0008, 3)  # 执行位置运动
```

### 8.3 控制多电机同步运动

```python
# 设置2号电机参数（基址0x0010）
client.write_registers(0x0012, [0, 1000, 20, 0, 3200, 0])  # 方向/速度/加速度/位置
client.write_register(0x0018, 3)  # 位置运动命令

# 设置3号电机参数（基址0x0020）
client.write_registers(0x0022, [1, 1000, 20, 0, 3200, 0])  # CCW方向
client.write_register(0x0028, 3)  # 位置运动命令

# 触发同步运动（通过1号电机执行）
client.write_register(0x0008, 9)  # CMD_SYNC_MOTION
```

### 8.4 回零操作

```python
# 配置回零参数
client.write_register(0x0009, 0)      # 回零模式=就近回零
client.write_register(0x000A, 500)    # 回零速度=500RPM
client.write_register(0x000B, 0)      # 超时高字节=0
client.write_register(0x000C, 10000)  # 超时低字节=10秒

# 执行回零
client.write_register(0x0008, 6)      # CMD_HOMING

# 轮询回零状态
while True:
    status = client.read_input_registers(0x0108, 1)
    if status.registers[0] == 2:  # 回零完成
        print("回零成功!")
        break
    time.sleep(0.1)
```

---

## 9. 错误代码表

| 错误码 | 说明 | 处理方法 |
|-------|------|---------|
| **0** | 正常 | - |
| **1** | 电机堵转 | 执行CMD_RESET_CLOG清除 |
| **2** | 回零超时 | 检查机械结构/增加超时时间 |
| **3** | 参数错误 | 检查速度/加速度范围 |
| **4** | 通信超时 | 检查RS485连接 |
| **5** | 编码器异常 | 检查编码器接线 |

---

## 10. 实现优先级建议

### 第一阶段（MVP）
- ✅ 实现0x03/0x06功能码（读写单个寄存器）
- ✅ 实现基础控制：使能、位置运动、停止
- ✅ 实现状态反馈：位置、速度、运行状态

### 第二阶段（完善）
- ✅ 实现0x10功能码（批量写入）
- ✅ 实现速度模式、回零功能
- ✅ 实现多电机同步

### 第三阶段（高级）
- ✅ 实现0x01/0x02线圈和离散输入
- ✅ 实现参数保存到Flash
- ✅ 实现异常诊断和错误恢复

---

**下一步**: 开始编写Modbus协议解析代码！
