# Y_V2协议迁移进度报告

**迁移日期**: 2025-12-03  
**状态**: 95%完成，待修复motor_monitor.c语法错误

## ✅ 已完成的工作

### 1. 协议驱动层（核心）
- ✅ 创建`Drivers/BSP/Y_V2/y_v2.h` (150行，完整API声明)
- ✅ 创建`Drivers/BSP/Y_V2/y_v2.c` (550行，完整实现)
- ✅ 实现所有X固件命令：
  * 触发动作：校准/重启/清零/解除保护/恢复出厂
  * 运动控制：使能/速度/位置(直通+梯形)/停止/同步
  * 回零控制：设置零点/触发回零/中断回零/读取参数/修改参数
  * 参数读写：读系统参数/修改ID/修改细分/修改电流/修改PID/读取全参数

### 2. 配置文件更新
- ✅ `app_config.h`新增角度换算系数：
  ```c
  #define DEGREES_PER_MM              18.0f    // 360°/20mm导程
  #define PULSES_TO_DEGREES(p)        ((p) * 360.0f / 3200.0f)
  #define DEGREES_TO_PULSES(d)        ((d) * 3200.0f / 360.0f)
  ```

### 3. 应用层更新（5个文件）
- ✅ `Core/App/motor_zdt.c`
  * 头文件: emm_v5.h → y_v2.h
  * 使能控制: Emm_V5_En_Control → Y_V2_En_Control
  * 位置控制: Emm_V5_Pos_Control → Y_V2_Bypass_Pos_Control (使用角度180.0f)

- ✅ `Core/App/printer_axis.c` (重点文件)
  * 头文件: emm_v5.h → y_v2.h
  * 使能控制: 4处替换
  * 位置控制: 9处替换，全部使用PULSES_TO_DEGREES宏转换
  * 同步运动: Emm_V5_Synchronous_motion → Y_V2_Synchronous_Motion
  * 回零命令: Emm_V5_Origin_Trigger_Return → Y_V2_Origin_Trigger_Return
  * 停止命令: Emm_V5_Stop_Now → Y_V2_Stop_Now

- ✅ `Drivers/Middlewares/USMART/usmart_config.c`
  * 头文件: emm_v5.h → y_v2.h
  * 函数注册: 8个Emm_V5系列 → 8个Y_V2系列
  * 更新签名: 支持float参数（USMART兼容浮点数）

- ✅ `Drivers/Middlewares/USMART/usmart_interface.c`
  * 头文件: emm_v5.h → y_v2.h
  * motor_enable: Emm_V5_En_Control → Y_V2_En_Control
  * 帮助文档: 更新命令示例（角度+加速度RPM/S）

- ⚠️ `Core/App/motor_monitor.c` (待修复)
  * 头文件: emm_v5.h → y_v2.h
  * 查询函数: Emm_V5_Read_Sys_Params → Y_V2_Read_Sys_Params (4处)
  * ❌ 存在语法错误: "expected declaration or statement at end of input"

### 4. 构建系统更新
- ✅ `CMakeLists.txt`
  * 源文件: Drivers/BSP/EMM_V5/emm_v5.c → Drivers/BSP/Y_V2/y_v2.c
  * 头文件路径: Drivers/BSP/EMM_V5 → Drivers/BSP/Y_V2
  * 其他3个文件保持不变（emm_uart.c, emm_state.c, emm_v5_parser.c）

- ✅ 文件整理
  * 删除EMM_V5目录（已重命名为Y_V2）
  * 移动官方例程到`Y_V2/official_reference/`（X_V2.c/h, board.c/h, usart.c/h）
  * 移动旧驱动到`Y_V2/official_reference/`（emm_v5.c/h）

## ❌ 待解决问题

### 问题1: motor_monitor.c编译错误
**错误类型**: 语法错误 - 大括号不匹配或嵌套函数定义  
**错误消息**:
```
error: expected declaration or statement at end of input
warning: ISO C forbids nested functions
error: invalid storage class for function 'find_motor_by_addr'
```

**可能原因**:
1. 某个函数缺少闭合大括号
2. PowerShell替换命令可能破坏了代码结构（使用了`Get-Content`/`Set-Content`）
3. 文件编码问题（显示"已上�?"等乱码字符）

**建议修复方案**:
1. 检查第165-200行的大括号匹配
2. 使用`git diff`或对比工具查看文件变化
3. 如果损坏严重，从备份恢复文件，手动替换函数调用

## 📋 迁移对照表

### 函数映射关系
```c
// 使能控制（签名不变）
Emm_V5_En_Control(addr, state, snF)
→ Y_V2_En_Control(addr, state, snF)

// 位置控制（关键变化）
Emm_V5_Pos_Control(addr, dir, speed, acc, pulses, raF, snF)  // 13字节，脉冲
→ Y_V2_Bypass_Pos_Control(addr, dir, speed_f, angle_f, raf, snF)  // 12字节，角度

// 速度控制（参数顺序变化）
Emm_V5_Vel_Control(addr, dir, speed, acc, snF)  // 8字节，acc在后
→ Y_V2_Vel_Control(addr, dir, acc, speed_f, snF)  // 9字节，acc在前

// 停止命令（签名不变）
Emm_V5_Stop_Now(addr, snF)
→ Y_V2_Stop_Now(addr, snF)

// 同步运动（大小写变化）
Emm_V5_Synchronous_motion(addr)
→ Y_V2_Synchronous_Motion(addr)

// 回零命令（签名不变）
Emm_V5_Origin_Trigger_Return(addr, mode, snF)
→ Y_V2_Origin_Trigger_Return(addr, mode, snF)

// 查询命令（参数类型变化）
Emm_V5_Read_Sys_Params(addr, uint8_t param)
→ Y_V2_Read_Sys_Params(addr, SysParams_t param)  // 枚举类型
```

### 单位转换公式
```c
// 脉冲 → 角度
float angle = PULSES_TO_DEGREES(pulses);  // pulses * 360 / 3200 = pulses * 0.1125
int32_t pulses = DEGREES_TO_PULSES(angle);  // angle * 3200 / 360 = angle * 8.8889

// 示例
3200脉冲 = 360° (1圈)
1600脉冲 = 180° (半圈)
160脉冲 = 18° (1mm at 20mm导程)
```

## 🔧 下一步行动

1. **立即**: 修复motor_monitor.c的语法错误
   - 检查大括号匹配
   - 验证函数定义结构
   - 测试编译通过

2. **编译测试**: 
   ```powershell
   cmake --build --preset Debug
   ```

3. **功能验证**: 通过USMART测试命令
   ```
   Y_V2_En_Control(1,1,0)                   # 使能电机1
   Y_V2_Bypass_Pos_Control(1,0,300.0,180.0,0,0)  # 转半圈
   Y_V2_Stop_Now(1,0)                        # 停止
   Y_V2_Read_Sys_Params(1,14)                # 查询转速(S_VEL=14)
   ```

4. **烧录测试**: 
   - 使用pyOCD或STM32CubeProgrammer烧录
   - 连接电机硬件
   - 测试实际响应

## 📊 代码统计

### Flash占用预估
- 旧Emm_V5驱动: ~2KB (简化版，仅位置/速度/使能)
- 新Y_V2驱动: ~3.5KB (完整版，25个API函数)
- **净增加**: ~1.5KB

### 修改文件清单
```
新建文件 (2个):
  ├── Drivers/BSP/Y_V2/y_v2.h (5KB, 150行)
  └── Drivers/BSP/Y_V2/y_v2.c (20KB, 550行)

修改文件 (6个):
  ├── Core/App/app_config.h (+4行角度系数)
  ├── Core/App/motor_zdt.c (3处替换)
  ├── Core/App/printer_axis.c (15处替换，关键文件)
  ├── Core/App/motor_monitor.c (5处替换，待修复)
  ├── Drivers/Middlewares/USMART/usmart_config.c (9处替换)
  └── Drivers/Middlewares/USMART/usmart_interface.c (2处替换+文档更新)

删除文件 (移动到official_reference/):
  ├── Drivers/BSP/Y_V2/emm_v5.c/h (旧驱动)
  ├── Drivers/BSP/Y_V2/X_V2.c/h (官方F407例程)
  ├── Drivers/BSP/Y_V2/board.c/h (官方F407初始化)
  └── Drivers/BSP/Y_V2/usart.c/h (官方F407串口)

配置文件 (1个):
  └── CMakeLists.txt (2处路径更新)
```

## 📝 参考文档
- `Docs/Y_V2_Migration_Map.md` - 完整迁移映射表
- `Docs/doc_Y57/README.md` - 电机技术手册
- `ZDT_X42S第二代闭环步进电机使用说明书V1.0.2_251118.pdf` - X固件协议定义

## ⚠️ 已知问题
1. ❌ motor_monitor.c语法错误（编译失败）
2. ⚠️ 未测试硬件响应（需要实际电机验证）
3. ⚠️ Y_V2响应帧解析器（emm_v5_parser.c）可能需要更新以支持X固件格式

---
**文档生成时间**: 2025-12-03 06:30  
**迁移完成度**: 95%  
**编译状态**: ❌ 失败（motor_monitor.c语法错误）
