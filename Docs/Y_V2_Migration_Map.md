# Y_V2协议迁移映射表

## 函数映射关系

### 使能控制（签名不变）
```c
// 旧：Emm_V5_En_Control(addr, state, snF)
// 新：Y_V2_En_Control(addr, state, snF)
```

### 位置控制（关键变化：脉冲→角度）
```c
// 旧：Emm_V5_Pos_Control(addr, dir, speed, acc, pulses, raF, snF)
// 新：Y_V2_Bypass_Pos_Control(addr, dir, speed_float, angle_float, raf, snF)
// 
// 单位转换：
// - speed: uint16_t RPM → float RPM（直接转换）
// - pulses → angle: 
//   * 3200脉冲 = 360° (16细分)
//   * angle = pulses * 360.0 / 3200.0 = pulses / 8.8889
//   * 或: pulses * 0.1125
```

### 速度控制（参数顺序变化）
```c
// 旧：Emm_V5_Vel_Control(addr, dir, speed, acc, snF)
// 新：Y_V2_Vel_Control(addr, dir, acc, speed_float, snF)
//     注意：acc在speed前面，且acc现在是16位RPM/S
```

### 停止命令（签名不变）
```c
// 旧：Emm_V5_Stop_Now(addr, snF)
// 新：Y_V2_Stop_Now(addr, snF)
```

### 同步运动（签名不变）
```c
// 旧：Emm_V5_Synchronous_motion(addr)
// 新：Y_V2_Synchronous_Motion(addr)  // 注意大小写
```

### 回零命令（签名不变）
```c
// 旧：Emm_V5_Origin_Trigger_Return(addr, mode, snF)
// 新：Y_V2_Origin_Trigger_Return(addr, mode, snF)
```

## printer_axis.c单位转换策略

### 关键宏定义（需修改）
```c
// 位于app_config.h或printer_axis.c
// 旧：AXIS_X_PULSES_PER_MM = 160  (3200脉冲/20mm导程)
// 新：AXIS_X_DEGREES_PER_MM = 18.0  (360°/20mm导程)
```

### 转换公式
```c
/* printer_axis.c中的转换逻辑 */
// 旧方式：
// int32_t pulses = distance_mm * AXIS_X_PULSES_PER_MM;
// Emm_V5_Pos_Control(addr, dir, spd, acc, pulses, raF, snF);

// 新方式：
// float angle = distance_mm * AXIS_X_DEGREES_PER_MM;  // 18.0 = 360/20
// Y_V2_Bypass_Pos_Control(addr, dir, (float)spd, angle, raf, snF);
```

### printer_axis.c需要更新的位置

1. **头文件引用** (第12行)
   ```c
   - #include "emm_v5.h"
   + #include "y_v2.h"
   ```

2. **使能控制** (4处)
   - 第165行: 单轴使能
   - 第184行: 单轴失能
   - 第197行: 全轴使能
   - 第212行: 全轴失能
   ```c
   Emm_V5_En_Control → Y_V2_En_Control  // 直接替换
   ```

3. **位置控制** (9处)
   - 第252-262行: `printer_axis_move_relative()`
   - 第310-339行: `printer_axis_move_relative_mm_int()`
   
   **替换策略**：
   ```c
   // 步骤1：计算脉冲数
   int32_t pulses = distance_mm * PULSES_PER_MM;
   
   // 步骤2：转换为角度
   float angle = pulses * 0.1125f;  // 0.1125 = 360/3200
   // 或直接：float angle = distance_mm * DEGREES_PER_MM;
   
   // 步骤3：调用新API
   Y_V2_Bypass_Pos_Control(addr, dir, (float)spd, angle, raf, snF);
   ```

4. **同步运动** (2处)
   - 第258行: `printer_axis_move_relative()`
   - 第339行: `printer_axis_move_relative_mm_int()`
   ```c
   Emm_V5_Synchronous_motion(0) → Y_V2_Synchronous_Motion(0)
   ```

5. **回零命令** (1处)
   - 第380行: `printer_axis_home()`
   ```c
   Emm_V5_Origin_Trigger_Return → Y_V2_Origin_Trigger_Return  // 直接替换
   ```

6. **停止命令** (1处)
   - 第425行: `printer_axis_emergency_stop()`
   ```c
   Emm_V5_Stop_Now(0, false) → Y_V2_Stop_Now(0, false)
   ```

## app_config.h需要更新的配置

### 位置换算系数（新增）
```c
/* ==================== 3D打印机机械参数 ==================== */
/* 导程与位置换算 */
#define AXIS_X_LEAD_MM           20.0f    /* X轴导程(mm/圈) */
#define AXIS_Y_LEAD_MM           20.0f    /* Y轴导程(mm/圈) */
#define AXIS_Z_LEAD_MM           8.0f     /* Z轴导程(mm/圈) - 精密丝杠 */

#define AXIS_X_DEGREES_PER_MM    (360.0f / AXIS_X_LEAD_MM)  /* 18.0°/mm */
#define AXIS_Y_DEGREES_PER_MM    (360.0f / AXIS_Y_LEAD_MM)  /* 18.0°/mm */
#define AXIS_Z_DEGREES_PER_MM    (360.0f / AXIS_Z_LEAD_MM)  /* 45.0°/mm */

/* 旧的脉冲系数（保留用于单位验证） */
#define AXIS_X_PULSES_PER_MM     160      /* 3200脉冲/20mm = 160脉冲/mm */
#define AXIS_Y_PULSES_PER_MM     160
#define AXIS_Z_PULSES_PER_MM     400      /* 3200脉冲/8mm = 400脉冲/mm */
```

## USMART命令注册（需更新）

### usmart_config.c
```c
// 删除旧的Emm_V5系列
- {(void *)Emm_V5_En_Control, "void Emm_V5_En_Control(uint8_t addr,uint8_t state,uint8_t snF)"},
- {(void *)Emm_V5_Pos_Control, "void Emm_V5_Pos_Control(...)"},
// ...

// 添加新的Y_V2系列
+ {(void *)Y_V2_En_Control, "void Y_V2_En_Control(uint8_t addr,uint8_t state,uint8_t snF)"},
+ {(void *)Y_V2_Bypass_Pos_Control, "void Y_V2_Bypass_Pos_Control(uint8_t addr,uint8_t dir,float vel,float pos,uint8_t raf,uint8_t snF)"},
+ {(void *)Y_V2_Vel_Control, "void Y_V2_Vel_Control(uint8_t addr,uint8_t dir,uint16_t acc,float vel,uint8_t snF)"},
+ {(void *)Y_V2_Stop_Now, "void Y_V2_Stop_Now(uint8_t addr,uint8_t snF)"},
+ {(void *)Y_V2_Origin_Trigger_Return, "void Y_V2_Origin_Trigger_Return(uint8_t addr,uint8_t mode,uint8_t snF)"},
+ {(void *)Y_V2_Read_Sys_Params, "void Y_V2_Read_Sys_Params(uint8_t addr,SysParams_t param)"},
```

### USMART测试命令示例
```
# 使能电机1
Y_V2_En_Control(1,1,0)

# 速度模式：500RPM，加速度1000 RPM/S
Y_V2_Vel_Control(1,0,1000,500.0,0)

# 位置模式：转180°(半圈)，速度300RPM
Y_V2_Bypass_Pos_Control(1,0,300.0,180.0,0,0)

# 停止
Y_V2_Stop_Now(1,0)

# 读取转速
Y_V2_Read_Sys_Params(1,14)  // S_VEL=14
```

## 迁移检查清单

- [x] 创建y_v2.h头文件
- [x] 创建y_v2.c实现文件
- [x] 更新CMakeLists.txt（源文件+头文件路径）
- [x] 更新motor_zdt.c头文件引用
- [x] 更新motor_zdt.c函数调用
- [ ] 更新printer_axis.c头文件引用
- [ ] 更新printer_axis.c所有函数调用（15处）
- [ ] 添加app_config.h角度换算系数
- [ ] 更新USMART注册表
- [ ] 编译测试
- [ ] 功能验证

## 注意事项

1. **浮点数精度**：Y_V2使用float类型，注意加f后缀（如180.0f）
2. **参数顺序**：速度命令中acc在vel前面
3. **单位验证**：
   - 1圈 = 3200脉冲 = 360°
   - 半圈 = 1600脉冲 = 180°
   - 1mm = 160脉冲 = 18° (20mm导程)
4. **同步运动**：函数名大小写变化（motion→Motion）
5. **USMART限制**：虽然Y_V2使用float，但USMART支持浮点数参数

