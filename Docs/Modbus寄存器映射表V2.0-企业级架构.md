# Modbus寄存器映射表 V2.0 - 企业级架构设计

## 版本信息
- **版本**: V2.0 Enterprise
- **日期**: 2025-12-01
- **设计目标**: 工业级多电机控制网关
- **支持电机数**: 16台（可扩展到32台）

---

## 设计理念

### 核心目标
构建**可扩展、高性能、易维护**的Modbus网关系统：
- ✅ 支持16台电机全功能控制
- ✅ 每电机32个直接寄存器 + 无限扩展参数
- ✅ 多段轨迹缓冲（64段）
- ✅ 间接寻址机制突破地址限制
- ✅ 事件驱动架构（中断通知）
- ✅ 参数保存/恢复到Flash

### V2.0相比V1.0的改进

| 特性 | V1.0 | V2.0 |
|------|------|------|
| 电机数量 | 8台 | 16台 |
| 每电机寄存器 | 16个 | 32个直接 + 无限间接 |
| 参数配置 | 仅基础控制 | PID/细分/电流/保护全覆盖 |
| 轨迹规划 | 无 | 64段缓冲 + 平滑过渡 |
| 扩展性 | 固定 | 间接寻址 + 自定义功能码 |
| 实时性 | 轮询 | 事件驱动 + 中断通知 |

---

## 1. Modbus通信参数

```
从机地址：   0x01 (STM32网关，可通过0x0001寄存器修改)
波特率：     115200 (与电机一致)
数据位：     8
停止位：     1
校验位：     无
CRC校验：    Modbus CRC16 (低字节在前)
帧间隔：     3.5字符时间 (≈300μs @115200)
```

---

## 2. 寄存器地址分配总览

```
地址范围          类型          功能区域                        数量    访问
═══════════════════════════════════════════════════════════════════════════════
【全局控制区】
0x0000-0x000F    Holding      系统配置(网关地址/波特率/错误码) 16     R/W
0x0010-0x001F    Holding      事件控制(中断使能/标志清除)      16     R/W
0x0020-0x003F    Holding      间接寻址控制器                  32     R/W
0x0040-0x005F    Holding      轨迹缓冲区控制                  32     R/W
0x0060-0x007F    Input        系统状态反馈                    32     R

【电机直接访问区】(16台×32寄存器=512寄存器)
0x0100-0x011F    Holding      1号电机控制参数                 32     R/W
0x0120-0x013F    Holding      2号电机控制参数                 32     R/W
0x0140-0x015F    Holding      3号电机控制参数                 32     R/W
0x0160-0x017F    Holding      4号电机控制参数                 32     R/W
0x0180-0x019F    Holding      5号电机控制参数                 32     R/W
0x01A0-0x01BF    Holding      6号电机控制参数                 32     R/W
0x01C0-0x01DF    Holding      7号电机控制参数                 32     R/W
0x01E0-0x01FF    Holding      8号电机控制参数                 32     R/W
0x0200-0x021F    Holding      9号电机控制参数                 32     R/W
0x0220-0x023F    Holding      10号电机控制参数                32     R/W
0x0240-0x025F    Holding      11号电机控制参数                32     R/W
0x0260-0x027F    Holding      12号电机控制参数                32     R/W
0x0280-0x029F    Holding      13号电机控制参数                32     R/W
0x02A0-0x02BF    Holding      14号电机控制参数                32     R/W
0x02C0-0x02DF    Holding      15号电机控制参数                32     R/W
0x02E0-0x02FF    Holding      16号电机控制参数                32     R/W

【电机状态反馈区】(16台×32寄存器=512寄存器)
0x0300-0x031F    Input        1号电机状态                     32     R
0x0320-0x033F    Input        2号电机状态                     32     R
...
0x04E0-0x04FF    Input        16号电机状态                    32     R

【间接访问区】
0x0500-0x051F    Holding      间接参数缓冲区                  32     R/W
0x0520-0x053F    Input        间接状态缓冲区                  32     R

【轨迹缓冲区】(64段×8参数=512寄存器)
0x0600-0x07FF    Holding      运动段缓冲(dir/vel/acc/pos...)  512    R/W

【线圈和离散量】
0x0800-0x081F    Coils        快速开关(电机使能/停止...)      32     R/W
0x0900-0x091F    Discrete     快速状态(运行中/到位/堵转...)    32     R
```

**寻址宏定义**：
```c
#define MOTOR_CTRL_BASE(id)   (0x0100 + ((id) - 1) * 0x20)  // 控制基址
#define MOTOR_STATUS_BASE(id) (0x0300 + ((id) - 1) * 0x20)  // 状态基址

// 示例：
// 1号电机控制: 0x0100-0x011F
// 1号电机状态: 0x0300-0x031F
// 8号电机控制: 0x01E0-0x01FF
// 16号电机状态: 0x04E0-0x04FF
```

---

## 3. 全局控制区详解 (0x0000-0x007F)

### 3.1 系统配置寄存器 (0x0000-0x000F)

| 地址 | 名称 | R/W | 默认值 | 说明 |
|------|------|-----|--------|------|
| **0x0000** | SYS_CTRL | R/W | 0x0000 | 系统控制<br>BIT0=全局使能, BIT1=紧急停止<br>BIT2=软复位, BIT8=保存参数到Flash |
| **0x0001** | GATEWAY_ADDR | R/W | 0x01 | 网关Modbus地址(1-247) |
| **0x0002** | MOTOR_COUNT | R | 动态 | 在线电机数量(1-16) |
| **0x0003** | SCAN_INTERVAL | R/W | 100 | 状态扫描周期(ms, 10-1000) |
| **0x0004** | MOTOR_BAUDRATE | R/W | 115200 | 电机通信波特率<br>0=9600, 1=115200, 2=256000 |
| **0x0005** | CMD_TIMEOUT | R/W | 1000 | 命令超时(ms, 100-5000) |
| **0x0006** | ERROR_CODE | R | 0x0000 | 全局错误码(见错误表) |
| **0x0007** | ERROR_MOTOR_ID | R | 0 | 发生错误的电机号(0=系统) |
| **0x0008** | FIRMWARE_VER | R | 0x0200 | 固件版本(V2.0 = 0x0200) |
| **0x0009** | HARDWARE_VER | R | 0x0103 | 硬件版本(STM32F103) |
| **0x000A** | UPTIME_H | R | - | 运行时间高16位(秒) |
| **0x000B** | UPTIME_L | R | - | 运行时间低16位(秒) |
| **0x000C** | RX_COUNT_H | R | - | 接收帧计数高位 |
| **0x000D** | RX_COUNT_L | R | - | 接收帧计数低位 |
| **0x000E** | TX_COUNT_H | R | - | 发送帧计数高位 |
| **0x000F** | TX_COUNT_L | R | - | 发送帧计数低位 |

### 3.2 事件控制寄存器 (0x0010-0x001F)

支持事件驱动编程，减少轮询开销。

| 地址 | 名称 | R/W | 说明 |
|------|------|-----|------|
| **0x0010** | EVENT_ENABLE | R/W | 事件使能<br>BIT0=到位中断, BIT1=堵转告警<br>BIT2=回零完成, BIT3=轨迹完成 |
| **0x0011** | EVENT_FLAGS | R | 事件标志位(读清除)<br>对应EVENT_ENABLE的位 |
| **0x0012** | EVENT_MOTOR_MASK_H | R/W | 监听电机掩码高8位(9-16号) |
| **0x0013** | EVENT_MOTOR_MASK_L | R/W | 监听电机掩码低8位(1-8号) |
| **0x0014** | ALARM_CODE | R | 告警代码(严重性<错误) |
| **0x0015** | ALARM_MOTOR_ID | R | 告警来源电机号 |
| **0x0016-0x001F** | RESERVED | - | 保留 |

**事件使用示例**：
```python
# 使能到位中断和堵转告警
client.write_register(0x0010, 0x0003)  # BIT0+BIT1

# 监听1-4号电机
client.write_register(0x0013, 0x000F)  # BIT0-3

# 主循环检查事件
while True:
    flags = client.read_holding_registers(0x0011, 1).registers[0]
    if flags & 0x01:  # 到位中断
        print("电机到达目标位置!")
    if flags & 0x02:  # 堵转告警
        motor_id = client.read_holding_registers(0x0015, 1).registers[0]
        print(f"电机{motor_id}堵转!")
```

### 3.3 间接寻址控制器 (0x0020-0x003F)

**突破寄存器地址限制，访问电机所有内部参数！**

| 地址 | 名称 | R/W | 说明 |
|------|------|-----|------|
| **0x0020** | INDEX_MOTOR_ID | R/W | 目标电机号(1-16, 0=广播) |
| **0x0021** | INDEX_PARAM_GROUP | R/W | 参数组选择<br>0=基础, 1=PID, 2=回零, 3=保护<br>4=编码器, 5=电流环, 6=高级, 7=用户自定义 |
| **0x0022** | INDEX_PARAM_ID | R/W | 参数ID(0-255) |
| **0x0023** | INDEX_VALUE_H | R/W | 参数值高16位(int32/float高半部) |
| **0x0024** | INDEX_VALUE_L | R/W | 参数值低16位(int32/float低半部) |
| **0x0025** | INDEX_OPERATION | W | 操作命令<br>1=读取, 2=写入, 3=保存到Flash<br>4=恢复默认, 5=锁定参数 |
| **0x0026** | INDEX_STATUS | R | 0=空闲, 1=执行中, 2=成功, 3=失败 |
| **0x0027** | INDEX_ERROR_CODE | R | 操作错误码 |
| **0x0028-0x003F** | INDEX_BATCH[24] | R/W | 批量间接访问缓冲区 |

**参数组详解**：

#### 组1：PID参数 (INDEX_PARAM_GROUP=1)
| ID | 参数名 | 单位 | 范围 | 说明 |
|----|--------|------|------|------|
| 0 | PID_P | - | 0-65535 | 位置环比例系数 |
| 1 | PID_I | - | 0-65535 | 位置环积分系数 |
| 2 | PID_D | - | 0-65535 | 位置环微分系数 |
| 3 | VEL_P | - | 0-65535 | 速度环P |
| 4 | VEL_I | - | 0-65535 | 速度环I |
| 5 | CUR_P | - | 0-65535 | 电流环P |
| 6 | CUR_I | - | 0-65535 | 电流环I |

#### 组2：回零参数 (INDEX_PARAM_GROUP=2)
| ID | 参数名 | 单位 | 说明 |
|----|--------|------|------|
| 0 | HOME_OFFSET | pulse | 回零后偏移量 |
| 1 | HOME_FINAL_VEL | RPM | 回零最终速度 |
| 2 | HOME_SWITCH_LEVEL | 0/1 | 限位开关触发电平 |

#### 组3：保护参数 (INDEX_PARAM_GROUP=3)
| ID | 参数名 | 单位 | 说明 |
|----|--------|------|------|
| 0 | OVER_TEMP | °C | 过温保护阈值 |
| 1 | OVER_VOLTAGE | mV | 过压保护阈值 |
| 2 | UNDER_VOLTAGE | mV | 欠压保护阈值 |
| 3 | OVER_CURRENT | mA | 过流保护阈值 |

**间接访问示例 - 修改1号电机PID_P参数**：
```python
# 方法1: 单次访问
client.write_register(0x0020, 1)      # 电机1
client.write_register(0x0021, 1)      # PID组
client.write_register(0x0022, 0)      # 参数0=P值
client.write_register(0x0024, 8000)   # 新值=8000
client.write_register(0x0025, 2)      # 执行写入
time.sleep(0.01)
status = client.read_holding_registers(0x0026, 1).registers[0]
if status == 2:
    print("PID_P修改成功!")

# 方法2: 批量访问(一次写入多个参数)
client.write_registers(0x0020, [1, 1, 0, 0, 8000, 2])  # 一次性写入全部参数
```

### 3.4 轨迹缓冲区控制 (0x0040-0x005F)

支持预加载64段运动指令，实现**无停顿连续运动**。

| 地址 | 名称 | R/W | 说明 |
|------|------|-----|------|
| **0x0040** | TRAJ_MOTOR_ID | R/W | 轨迹目标电机(1-16) |
| **0x0041** | TRAJ_BUFFER_SIZE | R | 缓冲区容量(默认64) |
| **0x0042** | TRAJ_WRITE_INDEX | R/W | 写入指针(0-63) |
| **0x0043** | TRAJ_READ_INDEX | R | 读取指针(当前执行段) |
| **0x0044** | TRAJ_FREE_COUNT | R | 剩余空闲槽位 |
| **0x0045** | TRAJ_USED_COUNT | R | 已缓冲段数 |
| **0x0046** | TRAJ_CTRL | W | 控制命令<br>1=开始, 2=暂停, 3=恢复<br>4=停止, 5=清空, 6=单步执行 |
| **0x0047** | TRAJ_STATUS | R | 0=空闲, 1=运行, 2=暂停<br>3=已完成, 4=错误 |
| **0x0048** | TRAJ_FLAGS | R/W | BIT0=循环播放, BIT1=平滑过渡<br>BIT2=自动清空, BIT3=完成中断 |
| **0x0049** | TRAJ_SMOOTH_TIME | R/W | 段间平滑时间(ms, 0-1000) |
| **0x004A** | TRAJ_ERROR_CODE | R | 轨迹错误码 |
| **0x004B** | TRAJ_LOOP_COUNT | R/W | 循环次数(0=无限) |
| **0x004C** | TRAJ_CURRENT_LOOP | R | 当前循环计数 |

**轨迹段定义 (0x0600+段号×8)**：
```
每段占用8个寄存器:
+0: DIRECTION      (0=CW, 1=CCW)
+1: VELOCITY       (RPM)
+2: ACCELERATION   (0-255)
+3: POSITION_H     (高16位)
+4: POSITION_L     (低16位)
+5: MOTION_TYPE    (0=相对, 1=绝对)
+6: DWELL_TIME     (到位后停留时间ms)
+7: FLAGS          (BIT0=等待到位, BIT1=同步触发)
```

**轨迹示例 - 正方形轨迹**：
```python
motor_id = 1
steps_per_side = 10000  # 每边10000脉冲

# 配置轨迹
client.write_register(0x0040, motor_id)  # 目标电机
client.write_register(0x0048, 0x01)      # 循环播放
client.write_register(0x0046, 5)         # 清空缓冲

# 写入4段轨迹(正方形)
segments = [
    # 段0: 右移
    [0, 1000, 20, 0, steps_per_side, 0, 0, 0x01],
    # 段1: 上移  
    [0, 1000, 20, 0, steps_per_side, 0, 0, 0x01],
    # 段2: 左移
    [1, 1000, 20, 0, steps_per_side, 0, 0, 0x01],
    # 段3: 下移
    [1, 1000, 20, 0, steps_per_side, 0, 500, 0x01],  # 最后停留500ms
]

for i, seg in enumerate(segments):
    base_addr = 0x0600 + i * 8
    client.write_registers(base_addr, seg)

# 启动轨迹
client.write_register(0x0046, 1)  # 开始执行

# 监控进度
while True:
    status = client.read_holding_registers(0x0047, 1).registers[0]
    current_seg = client.read_holding_registers(0x0043, 1).registers[0]
    print(f"状态: {status}, 当前段: {current_seg}")
    if status == 3:  # 完成
        break
    time.sleep(0.1)
```

---

## 4. 电机控制寄存器详解 (每电机32个)

### 4.1 1号电机示例 (0x0100-0x011F)

#### 基础运动控制 (0x0100-0x010F)

| 偏移 | 地址 | 名称 | R/W | 范围 | 默认 | 说明 |
|-----|------|------|-----|------|------|------|
| +0 | 0x0100 | ENABLE | R/W | 0/1 | 0 | 电机使能 |
| +1 | 0x0101 | MOTION_MODE | R/W | 0-2 | 0 | 0=停止, 1=位置, 2=速度 |
| +2 | 0x0102 | DIRECTION | R/W | 0/1 | 0 | 0=CW, 1=CCW |
| +3 | 0x0103 | TARGET_SPEED | R/W | 0-5000 | 1000 | 目标速度(RPM) |
| +4 | 0x0104 | ACCELERATION | R/W | 0-255 | 20 | 加速度(0=直接启动) |
| +5 | 0x0105 | TARGET_POS_H | R/W | 0-65535 | 0 | 目标位置高16位 |
| +6 | 0x0106 | TARGET_POS_L | R/W | 0-65535 | 0 | 目标位置低16位 |
| +7 | 0x0107 | MOTION_TYPE | R/W | 0/1 | 0 | 0=相对, 1=绝对 |
| +8 | 0x0108 | EXEC_COMMAND | W | 0-20 | 0 | 执行命令(见命令表) |
| +9 | 0x0109 | SYNC_FLAG | R/W | 0/1 | 0 | 同步运动标志 |
| +10 | 0x010A | SUBDIVIDE | R/W | 8/16/32 | 16 | 细分数(重启生效) |
| +11 | 0x010B | CURRENT_LIMIT | R/W | 0-5000 | 2000 | 电流限制(mA) |
| +12 | 0x010C | SMOOTH_FACTOR | R/W | 0-100 | 50 | S曲线平滑系数(%) |
| +13 | 0x010D | JERK_LIMIT | R/W | 0-255 | 100 | 加加速度限制 |
| +14 | 0x010E | STOP_MODE | R/W | 0-2 | 0 | 0=减速停止,1=急停,2=自由停 |
| +15 | 0x010F | CTRL_MODE | R/W | 1-2 | 2 | 1=开环, 2=闭环 |

#### 回零参数 (0x0110-0x0117)

| 偏移 | 地址 | 名称 | R/W | 默认 | 说明 |
|-----|------|------|-----|------|------|
| +16 | 0x0110 | HOMING_MODE | R/W | 0 | 0=就近, 1=方向, 2=碰撞, 3=限位 |
| +17 | 0x0111 | HOMING_DIR | R/W | 0 | 0=CW, 1=CCW |
| +18 | 0x0112 | HOMING_SPEED | R/W | 500 | 回零速度(RPM) |
| +19 | 0x0113 | HOMING_TIMEOUT_H | R/W | 0 | 超时高16位(ms) |
| +20 | 0x0114 | HOMING_TIMEOUT_L | R/W | 10000 | 超时低16位(ms) |
| +21 | 0x0115 | STALL_VEL | R/W | 10 | 堵转检测速度(RPM) |
| +22 | 0x0116 | STALL_CURRENT | R/W | 3000 | 堵转检测电流(mA) |
| +23 | 0x0117 | STALL_TIME | R/W | 500 | 堵转检测时间(ms) |

#### 高级参数 (0x0118-0x011F)

| 偏移 | 地址 | 名称 | R/W | 说明 |
|-----|------|------|-----|------|
| +24 | 0x0118 | ENCODER_PPR | R/W | 编码器线数(只读) |
| +25 | 0x0119 | GEAR_RATIO_NUM | R/W | 齿轮比分子 |
| +26 | 0x011A | GEAR_RATIO_DEN | R/W | 齿轮比分母 |
| +27 | 0x011B | POS_TOLERANCE | R/W | 到位容差(脉冲) |
| +28 | 0x011C | MAX_SPEED_LIMIT | R/W | 最大速度限制(RPM) |
| +29 | 0x011D | MIN_SPEED_LIMIT | R/W | 最小速度限制(RPM) |
| +30 | 0x011E | USER_PARAM | R/W | 用户自定义参数 |
| +31 | 0x011F | MOTOR_RS485_ADDR | R/W | 电机RS485地址(1-255) |

### 4.2 执行命令表 (EXEC_COMMAND寄存器)

| 命令码 | 命令名 | 说明 | 对应API |
|-------|--------|------|---------|
| **0** | CMD_NONE | 无操作 | - |
| **1** | CMD_ENABLE | 使能电机 | `Emm_V5_En_Control(addr, true, snF)` |
| **2** | CMD_DISABLE | 失能电机 | `Emm_V5_En_Control(addr, false, snF)` |
| **3** | CMD_POS_MOVE | 位置运动 | `Emm_V5_Pos_Control(...)` |
| **4** | CMD_VEL_MOVE | 速度运动 | `Emm_V5_Vel_Control(...)` |
| **5** | CMD_STOP_NOW | 立即停止 | `Emm_V5_Stop_Now(...)` |
| **6** | CMD_HOMING | 触发回零 | `Emm_V5_Origin_Trigger_Return(...)` |
| **7** | CMD_RESET_POS | 位置清零 | `Emm_V5_Reset_CurPos_To_Zero(...)` |
| **8** | CMD_RESET_CLOG | 解除堵转 | `Emm_V5_Reset_Clog_Pro(...)` |
| **9** | CMD_SYNC_MOTION | 同步运动触发(广播) | `Emm_V5_Synchronous_motion(0)` |
| **10** | CMD_SAVE_PARAMS | 保存参数到电机Flash | 内部EEPROM |
| **11** | CMD_LOAD_PARAMS | 恢复默认参数 | 内部操作 |
| **12** | CMD_CALIBRATE | 启动自动校准 | 编码器校准 |
| **13** | CMD_SET_ORIGIN | 设置当前位置为零点 | `Emm_V5_Origin_Set_O(...)` |
| **14** | CMD_INTERRUPT_HOME | 中断回零 | `Emm_V5_Origin_Interrupt(...)` |
| **15** | CMD_READ_VERSION | 读取固件版本 | `Emm_V5_Read_Sys_Params(S_VER)` |

---

## 5. 电机状态寄存器详解 (每电机32个只读)

### 5.1 1号电机状态示例 (0x0300-0x031F)

#### 实时状态 (0x0300-0x030F)

| 偏移 | 地址 | 名称 | 范围 | 说明 |
|-----|------|------|------|------|
| +0 | 0x0300 | STATUS | 0-7 | 运行状态<br>0=停止, 1=加速, 2=匀速, 3=减速<br>4=回零中, 5=堵转, 6=错误 |
| +1 | 0x0301 | CURRENT_POS_H | -32768~32767 | 当前位置高16位(有符号) |
| +2 | 0x0302 | CURRENT_POS_L | 0-65535 | 当前位置低16位 |
| +3 | 0x0303 | CURRENT_SPEED | 0-5000 | 当前速度(RPM) |
| +4 | 0x0304 | POS_ERROR_H | -32768~32767 | 位置误差高16位 |
| +5 | 0x0305 | POS_ERROR_L | 0-65535 | 位置误差低16位 |
| +6 | 0x0306 | TARGET_POS_H | -32768~32767 | 目标位置高16位 |
| +7 | 0x0307 | TARGET_POS_L | 0-65535 | 目标位置低16位 |
| +8 | 0x0308 | BUS_VOLTAGE | 0-65535 | 总线电压(mV) |
| +9 | 0x0309 | BUS_CURRENT | 0-65535 | 总线电流(mA) |
| +10 | 0x030A | MOTOR_TEMP | 0-200 | 电机温度(°C) |
| +11 | 0x030B | ENCODER_RAW_H | 0-65535 | 编码器原始值高位 |
| +12 | 0x030C | ENCODER_RAW_L | 0-65535 | 编码器原始值低位 |
| +13 | 0x030D | ENCODER_SPEED | 0-65535 | 编码器速度(计数/秒) |
| +14 | 0x030E | PWM_DUTY | 0-1000 | PWM占空比(千分比) |
| +15 | 0x030F | LOAD_RATE | 0-100 | 负载率(%) |

#### 标志位 (0x0310-0x0317)

| 偏移 | 地址 | 名称 | 说明 |
|-----|------|------|------|
| +16 | 0x0310 | FLAGS | BIT0=使能, BIT1=运行中, BIT2=到位<br>BIT3=回零完成, BIT4=堵转, BIT5=过流<br>BIT6=过压, BIT7=欠压, BIT8=过温 |
| +17 | 0x0311 | HOMING_STATUS | 0=未回零, 1=回零中, 2=成功, 3=失败 |
| +18 | 0x0312 | ERROR_CODE | 0-255 | 错误代码(0=正常) |
| +19 | 0x0313 | WARNING_CODE | 0-255 | 警告代码 |
| +20 | 0x0314 | FIRMWARE_VER_H | 0-255 | 电机固件主版本 |
| +21 | 0x0315 | FIRMWARE_VER_L | 0-255 | 电机固件次版本 |
| +22 | 0x0316 | COMM_ERROR_CNT | 0-65535 | 通信错误计数 |
| +23 | 0x0317 | RUNTIME_H | 0-65535 | 累计运行时间高位(分钟) |

#### 诊断信息 (0x0318-0x031F)

| 偏移 | 地址 | 名称 | 说明 |
|-----|------|------|------|
| +24 | 0x0318 | RUNTIME_L | 累计运行时间低位(分钟) |
| +25 | 0x0319 | MOVE_COUNT_H | 运动次数高位 |
| +26 | 0x031A | MOVE_COUNT_L | 运动次数低位 |
| +27 | 0x031B | STALL_COUNT | 堵转次数 |
| +28 | 0x031C | ALARM_HISTORY | 历史告警位图 |
| +29 | 0x031D | LAST_ERROR | 上次错误码 |
| +30 | 0x031E | RESERVED | 保留 |
| +31 | 0x031F | CHECKSUM | 状态校验和(可选) |

---

## 6. 线圈寄存器 (Coils - 单比特控制)

**地址0x0800-0x081F (32个布尔量)**

| 地址 | 名称 | R/W | 说明 |
|------|------|-----|------|
| **0x0800-0x080F** | MOTOR_ENABLE[1-16] | R/W | 电机1-16使能开关 |
| **0x0810** | GLOBAL_ENABLE | R/W | 全局使能(所有电机) |
| **0x0811** | EMERGENCY_STOP | W | 紧急停止(写1触发) |
| **0x0812** | AUTO_SCAN | R/W | 自动扫描电机状态 |
| **0x0813-0x081F** | USER_BITS[1-13] | R/W | 用户自定义位 |

**优势**：使用0x05功能码可以单比特操作，效率高。

```python
# 快速使能1号电机
client.write_coil(0x0800, True)

# 紧急停止所有电机
client.write_coil(0x0811, True)
```

---

## 7. 离散输入 (Discrete Inputs - 单比特状态)

**地址0x0900-0x091F (32个只读布尔量)**

| 地址 | 名称 | 说明 |
|------|------|------|
| **0x0900-0x090F** | MOTOR_RUNNING[1-16] | 电机1-16运行中 |
| **0x0910-0x091F** | MOTOR_AT_POS[1-16] | 电机1-16到位标志 |

---

## 8. Modbus功能码支持

| 功能码 | 名称 | 支持 | 说明 |
|-------|------|------|------|
| **0x01** | Read Coils | ✅ | 读线圈(开关量) |
| **0x02** | Read Discrete Inputs | ✅ | 读离散输入(只读位) |
| **0x03** | Read Holding Registers | ✅ | 读保持寄存器(可读写) |
| **0x04** | Read Input Registers | ✅ | 读输入寄存器(只读) |
| **0x05** | Write Single Coil | ✅ | 写单个线圈 |
| **0x06** | Write Single Register | ✅ | 写单个寄存器 |
| **0x0F** | Write Multiple Coils | ✅ | 批量写线圈 |
| **0x10** | Write Multiple Registers | ✅ | 批量写寄存器 |
| **0x17** | Read/Write Multiple Registers | ✅ | 读写组合(原子操作) |
| **0x2B** | Encapsulated Interface | 可选 | 自定义功能扩展 |

---

## 9. 错误代码表

### 系统错误 (0x0006寄存器)

| 错误码 | 名称 | 说明 | 处理方法 |
|-------|------|------|---------|
| **0x00** | NO_ERROR | 正常 | - |
| **0x01** | MODBUS_CRC_ERROR | CRC校验错误 | 检查通信质量 |
| **0x02** | MODBUS_TIMEOUT | 通信超时 | 检查线缆连接 |
| **0x03** | INVALID_REGISTER | 无效寄存器地址 | 检查地址范围 |
| **0x04** | INVALID_VALUE | 参数值越界 | 检查数据范围 |
| **0x05** | MOTOR_OFFLINE | 电机离线 | 检查电机电源/地址 |
| **0x06** | BUFFER_OVERFLOW | 缓冲区溢出 | 减少数据发送频率 |
| **0x10** | MOTOR_STALL | 电机堵转 | 解除堵转+检查负载 |
| **0x11** | OVER_CURRENT | 过流保护 | 降低电流限制 |
| **0x12** | OVER_VOLTAGE | 过压保护 | 检查电源电压 |
| **0x13** | UNDER_VOLTAGE | 欠压保护 | 提升电源电压 |
| **0x14** | OVER_TEMP | 过温保护 | 降低速度/加强散热 |
| **0x20** | HOMING_FAILED | 回零失败 | 检查机械结构 |
| **0x21** | TRAJ_ERROR | 轨迹错误 | 检查轨迹参数 |

### 电机错误 (0x0312寄存器)

| 错误码 | 说明 |
|-------|------|
| **0x00** | 正常 |
| **0x01** | 编码器异常 |
| **0x02** | 相电流异常 |
| **0x03** | 位置偏差过大 |
| **0x04** | 速度超限 |
| **0x05** | 加速度超限 |

---

## 10. 典型应用场景示例

### 场景1：单电机精确定位

```python
from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(port='COM3', baudrate=115200)

# 使能1号电机
client.write_register(0x0100, 1)  # ENABLE=1

# 配置位置运动
client.write_registers(0x0102, [
    0,      # DIRECTION=CW
    1000,   # TARGET_SPEED=1000RPM
    30,     # ACCELERATION=30
    0,      # TARGET_POS_H=0
    3200,   # TARGET_POS_L=3200 (1圈)
    0       # MOTION_TYPE=相对
])

# 执行运动
client.write_register(0x0108, 3)  # CMD_POS_MOVE

# 等待到位
while True:
    flags = client.read_input_registers(0x0310, 1).registers[0]
    if flags & 0x04:  # BIT2=到位
        print("到达目标位置!")
        break
    time.sleep(0.05)
```

### 场景2：多电机同步运动

```python
# 配置1-4号电机相同参数
for motor_id in range(1, 5):
    base = 0x0100 + (motor_id - 1) * 0x20
    client.write_registers(base + 0x02, [0, 1000, 20, 0, 3200, 0])  # 参数
    client.write_register(base + 0x09, 1)  # SYNC_FLAG=1
    client.write_register(base + 0x08, 3)  # CMD_POS_MOVE

# 广播同步触发
client.write_register(0x0108, 9)  # CMD_SYNC_MOTION (任意电机触发)
```

### 场景3：间接访问修改PID

```python
# 读取当前PID_P值
client.write_registers(0x0020, [1, 1, 0])  # 电机1, PID组, 参数0
client.write_register(0x0025, 1)  # 命令1=读取
time.sleep(0.01)
old_p = (client.read_holding_registers(0x0023, 2).registers[0] << 16) | \
         client.read_holding_registers(0x0024, 1).registers[0]
print(f"当前P值: {old_p}")

# 修改为新值
client.write_registers(0x0023, [0, 10000])  # 新值=10000
client.write_register(0x0025, 2)  # 命令2=写入
time.sleep(0.01)

# 保存到Flash
client.write_register(0x0025, 3)  # 命令3=保存
```

### 场景4：连续轨迹运动

```python
# 配置圆形轨迹(16段近似)
motor_id = 1
radius = 5000  # 半径脉冲数
segments = 16

client.write_register(0x0040, motor_id)  # 轨迹电机
client.write_register(0x0048, 0x03)  # 循环+平滑
client.write_register(0x0046, 5)  # 清空缓冲

for i in range(segments):
    angle = 2 * math.pi * i / segments
    dx = int(radius * math.cos(angle))
    dy = int(radius * math.sin(angle))
    
    base = 0x0600 + i * 8
    client.write_registers(base, [
        0 if dx >= 0 else 1,  # 方向
        1500,  # 速度
        20,    # 加速度
        0, abs(dx),  # 位置
        0,     # 相对运动
        0,     # 不停留
        0x01   # 等待到位
    ])

# 启动轨迹
client.write_register(0x0046, 1)  # 开始
```

### 场景5：事件驱动编程

```python
# 配置事件中断
client.write_register(0x0010, 0x0F)  # 使能所有事件
client.write_register(0x0013, 0xFFFF)  # 监听所有电机

# 主循环
while True:
    # 检查事件标志
    flags = client.read_holding_registers(0x0011, 1).registers[0]
    
    if flags & 0x01:  # 到位中断
        motor_id = client.read_holding_registers(0x0015, 1).registers[0]
        print(f"电机{motor_id}到位!")
        
    if flags & 0x02:  # 堵转告警
        motor_id = client.read_holding_registers(0x0015, 1).registers[0]
        print(f"电机{motor_id}堵转,正在恢复...")
        # 解除堵转
        base = 0x0100 + (motor_id - 1) * 0x20
        client.write_register(base + 0x08, 8)  # CMD_RESET_CLOG
        
    if flags & 0x04:  # 回零完成
        print("回零完成,开始正常运动")
        
    if flags & 0x08:  # 轨迹完成
        print("轨迹执行完成!")
        
    time.sleep(0.01)  # 事件驱动,可以很快响应
```

---

## 11. 性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| **响应时间** | <5ms | Modbus命令到电机响应 |
| **轨迹段切换** | <2ms | 无停顿平滑过渡 |
| **状态刷新率** | 100Hz | 默认10ms扫描周期 |
| **通信速率** | 115200bps | 可切换到256000bps |
| **最大电机数** | 16台 | 可扩展到32台 |
| **轨迹缓冲** | 64段 | 可配置更大 |
| **参数数量** | 1000+ | 通过间接访问 |

---

## 12. 实现优先级建议

### 第一阶段 (MVP - 最小可行产品)
- ✅ 实现0x03/0x06功能码
- ✅ 实现基础控制: 使能、位置、速度、停止
- ✅ 实现状态反馈: 位置、速度、运行状态
- ✅ 支持1-4台电机

### 第二阶段 (完善功能)
- ✅ 实现0x10批量写入
- ✅ 实现回零功能
- ✅ 实现间接寻址(PID/保护参数)
- ✅ 支持8台电机

### 第三阶段 (高级特性)
- ✅ 实现轨迹缓冲
- ✅ 实现事件机制
- ✅ 实现0x01/0x05线圈控制
- ✅ 支持16台电机

### 第四阶段 (企业级)
- ✅ Flash参数保存
- ✅ 错误诊断和恢复
- ✅ 数据记录和分析
- ✅ OTA固件升级

---

## 总结

**V2.0架构优势**：
1. ✅ **可扩展**: 间接寻址突破地址限制
2. ✅ **高性能**: 轨迹缓冲实现连续运动
3. ✅ **易维护**: 分层设计清晰
4. ✅ **企业级**: 完整的错误处理和诊断

**这套设计可以支撑复杂的工业应用场景！**

---

**下一步**: 开始编写Modbus协议解析核心代码！
