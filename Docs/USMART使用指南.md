# USMART 串口调试组件使用指南

## 功能概述

USMART（正点原子串口调试组件）允许通过串口助手调用STM32程序中的任意C函数，支持实时调试和测试电机控制功能，无需重新编译烧录。

**核心特性**:
- 串口交互式调用C函数（最多10个参数）
- 支持10进制/16进制参数输入和显示
- 函数执行时间统计（0.1ms精度）
- 自动命令解析和参数类型转换
- 内置帮助系统和错误提示

---

## 硬件连接

**串口配置**: USART1 (PA9/PA10)
- **波特率**: 115200
- **数据位**: 8
- **停止位**: 1
- **校验位**: 无
- **流控**: 无

**连接方式**: 使用USB转TTL模块连接到USART1
```
PA9 (TX) → RX (USB转TTL模块)
PA10(RX) → TX (USB转TTL模块)
GND      → GND
```

---

## 基础使用

### 1. 查看命令列表

在串口助手中输入 `?` 或 `help`，按回车:

```
? <回车>
```

**输出示例**:
```
read_addr(uint32_t addr)
write_addr(uint32_t addr,uint32_t val)
delay_ms(uint16_t nms)
delay_us(uint32_t nus)
led_init(void)
key_init(void)
key_scan(uint8_t mode)
iwdg_init(uint8_t prer,uint16_t rlr)
iwdg_feed(void)
motor_enable(uint8_t addr,uint8_t enable)
motor_pos_move(uint8_t addr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses)
motor_vel_move(uint8_t addr,uint8_t dir,uint16_t speed,uint8_t acc)
motor_stop(uint8_t addr)
motor_home(uint8_t addr)
motor_read_status(uint8_t addr)
```

### 2. 调用函数语法

**格式**: `函数名(参数1, 参数2, ...)`

**示例**:
```c
delay_ms(1000)          // 延时1000ms
led_init()              // 初始化LED（无参数函数也需要括号）
motor_enable(1,1)       // 使能地址为1的电机
```

**注意事项**:
- 参数之间用英文逗号分隔
- 字符串参数用双引号包裹（如果函数支持）
- 函数名区分大小写
- 输入完毕后按回车执行

### 3. 切换参数显示格式

**十进制显示**:
```
dec <回车>
```

**十六进制显示**:
```
hex <回车>
```

**数值转换**:
```
hex 100     → 显示: HEX 0X64
dec 0X64    → 显示: DEC 100
```

---

## 电机控制实战

### 场景1: 初次上电测试

```
motor_enable(1,1)                    # 使能地址1的电机
delay_ms(500)                        # 等待电机使能稳定
motor_pos_move(1,0,300,10,3200)      # 电机顺时针转1圈
```

**参数说明**:
- `motor_pos_move(地址, 方向, 速度RPM, 加速度, 脉冲数)`
- 方向: 0=顺时针(CW), 1=逆时针(CCW)
- 3200脉冲 = 1圈（16细分）

### 场景2: 连续速度调试

```
motor_vel_move(1,0,500,20)           # 启动电机: 500RPM
delay_ms(3000)                       # 运行3秒
motor_stop(1)                        # 急停
```

### 场景3: 回零操作

```
motor_home(1)                        # 电机1执行回零（模式0：单圈就近回零）
delay_ms(5000)                       # 等待回零完成
motor_read_status(1)                 # 读取电机状态
```

### 场景4: 多机同步（需先修改电机地址）

```
motor_enable(1,1)                    # 使能电机1
motor_enable(2,1)                    # 使能电机2
delay_ms(500)
motor_pos_move(1,0,300,10,1600)      # 电机1半圈
motor_pos_move(2,1,300,10,1600)      # 电机2反向半圈
```

---

## 高级功能

### 内存读写（仅调试时使用）

**读取地址内容**:
```
read_addr(0x20000000)                # 读取RAM起始地址的值
```

**写入地址**:
```
write_addr(0x40021018,0x00000010)    # 写入GPIOC_BSRR寄存器
```

**危险警告**: 错误的地址写入可能导致系统崩溃！

### 函数执行时间统计

开启时间统计后，每次函数执行会显示耗时:

```
runtime 1                            # 开启时间统计
motor_pos_move(1,0,300,10,3200)
# 输出: Function Run Time:12.3ms
runtime 0                            # 关闭时间统计
```

---

## 添加自定义函数

### 步骤1: 在`app_functions.h`声明函数

```c
// Core/App/app_functions.h
static inline void my_custom_test(uint8_t param1, uint16_t param2)
{
    printf("Custom test: %d, %d\r\n", param1, param2);
}
```

### 步骤2: 注册到USMART

编辑`Drivers/Middlewares/USMART/usmart_config.c`:

```c
struct _m_usmart_nametab usmart_nametab[] =
{
    // ... 现有函数
    
    /* 自定义测试函数 */
    {(void *)my_custom_test, "void my_custom_test(uint8_t param1,uint16_t param2)"},
};
```

### 步骤3: 重新编译

```powershell
cmake --build --preset Debug
```

### 步骤4: 测试新函数

```
my_custom_test(10,500)
```

**注意事项**:
- 函数签名字符串必须与实际函数声明完全一致
- 参数类型、顺序、个数必须匹配
- 最多支持10个参数
- 返回值会自动显示（非void函数）

---

## 常见问题排查

### 问题1: 输入命令无响应

**检查项**:
1. 串口连接是否正常（TX-RX交叉连接）
2. 波特率是否为115200
3. 串口助手是否开启"回车换行"（\r\n）
4. USMART是否已在`main.c`中初始化

**验证方法**:
```c
printf("USMART Test\r\n");  // 看能否收到调试输出
```

### 问题2: 提示"NOFUNCFIND"

**原因**: 函数名拼写错误或函数未注册

**解决方法**:
1. 输入`?`查看可用函数列表
2. 确认函数名大小写无误
3. 检查`usmart_config.c`是否已注册

### 问题3: 提示"PARMERR"（参数错误）

**原因**: 参数类型或个数不匹配

**示例错误**:
```
motor_pos_move(1,0,300)              # 缺少参数（需要5个）
motor_enable(1)                      # 缺少参数（需要2个）
```

**正确示例**:
```
motor_pos_move(1,0,300,10,3200)      # 5个参数完整
motor_enable(1,1)                    # 2个参数完整
```

### 问题4: 电机无响应

**检查项**:
1. 是否先调用`motor_enable(1,1)`使能
2. RS485 A-B线是否正确连接（A-A, B-B）
3. 电机地址是否为1（出厂默认）
4. 电机电源是否接通

**测试序列**:
```
motor_enable(1,1)                    # 使能，观察电机是否有"嗒"声
delay_ms(500)
motor_pos_move(1,0,100,5,1600)       # 低速半圈测试
```

---

## 技术原理

### 工作流程

1. **TIM4定时扫描** (10ms周期)
   - 定时器中断调用`usmart_dev.scan()`
   - 检查USART1接收缓冲区`g_usart1_rx_buf`

2. **命令解析** (`usmart_cmd_rec()`)
   - 提取函数名和参数
   - 在函数表`usmart_nametab[]`中查找匹配
   - 解析参数类型（数字/字符串）

3. **函数执行** (`usmart_exe()`)
   - 根据参数个数调用对应函数指针
   - 支持0-10个参数的任意组合
   - 返回值自动转换为字符串输出

4. **时间统计** (可选)
   - TIM4计数器记录函数执行前后的CNT值
   - 精度0.1ms，最大测量约13秒

### 内存占用

```
FLASH: 约4-6KB (取决于USMART_USE_HELP和USMART_USE_WRFUNS配置)
SRAM:  72字节基础 + PARM_LEN (当前200字节)
总计:  约272字节RAM
```

### 关键宏定义 (`usmart_port.h`)

```c
#define MAX_FNAME_LEN    30     // 函数名最大长度
#define MAX_PARM         10     // 最大参数个数
#define PARM_LEN         200    // 参数缓冲区长度
#define USMART_ENTIMX_SCAN 1    // 使能TIM4定时扫描
#define USMART_USE_HELP  1      // 使能帮助信息
#define USMART_USE_WRFUNS 1     // 使能内存读写函数
```

---

## 最佳实践

1. **调试流程规范**
   - 先输入`?`查看可用函数
   - 按步骤执行初始化序列（使能→配置→运动）
   - 使用`delay_ms()`添加观察间隔

2. **参数调优**
   - 速度从低到高测试（100→300→500 RPM）
   - 加速度从小到大（5→10→20）
   - 脉冲数按细分计算（3200=1圈）

3. **安全注意**
   - 首次测试使用小脉冲数（如1600=半圈）
   - 测试速度模式时准备好`motor_stop(1)`命令
   - 避免在生产代码中保留`write_addr()`调用

4. **代码集成**
   - 验证成功的参数可直接复制到C代码
   - 使用USMART快速验证新功能逻辑
   - 生产环境可通过宏关闭USMART节省资源

---

## 扩展资源

- **原理详解**: `Drivers/Middlewares/USMART/readme.txt`
- **函数注册表**: `Drivers/Middlewares/USMART/usmart_config.c`
- **移植说明**: `Drivers/Middlewares/USMART/usmart_port.h`
- **电机API**: `Drivers/BSP/EMM_V5/emm_v5.h`

---

*更新时间: 2025-12-01*  
*适用版本: STM32_485 V2.0*
