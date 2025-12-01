# STM32_485 项目架构规范 (V3.1)

## 🎯 核心设计原则

### 分层职责清晰
```
App层         → 业务编排（WHAT to do）
Middlewares层 → 功能实现（HOW to do）
BSP层         → 硬件抽象（接口封装）
SYSTEM层      → 基础设施（delay, usart等）
```

### 关键规则

1. **App层只做编排，不做实现**
   - ✅ 调用函数、流程控制、状态机
   - ❌ 算法实现、数据结构、复杂逻辑

2. **功能实现放Middlewares层**
   - ✅ 协议栈（Modbus、ModemManager）
   - ✅ 算法库（PID、滤波器）
   - ✅ 可复用组件（Multi-Motor Manager）

3. **BSP层只封装硬件**
   - ✅ GPIO操作（LED、KEY）
   - ✅ 通信协议（EMM_V5、RS485）
   - ❌ 业务逻辑（不要在LED驱动里做状态机）

---

## 📂 正确的目录结构

```
STM32_485/
├── Core/App/                          # 应用层（业务编排）
│   ├── main.c                         # 主程序入口（初始化+主循环）
│   ├── app_init.c                     # 系统初始化编排
│   ├── motor_zdt.c                    # 电机控制业务（按键→命令）
│   ├── modbus_task.c                  # Modbus任务调度
│   ├── modbus_adapter.c               # Modbus回调适配器
│   └── diagnostic.c                   # 系统诊断业务
│
├── Drivers/Middlewares/               # 中间件层（可重用功能实现）
│   ├── MODBUS/                        # Modbus协议栈
│   │   ├── modbus_rtu.c               # RTU协议解析
│   │   ├── modbus_gateway.c           # 寄存器映射网关
│   │   └── modbus_hal.c               # 硬件抽象层
│   ├── MULTI_MOTOR/                   # ✅ 多电机管理中间件（正确位置）
│   │   ├── multi_motor_manager.c      # 电机发现、批量控制、状态监控
│   │   └── multi_motor_manager.h      
│   ├── USMART/                        # 串口调试组件
│   │   ├── usmart.c                   # 命令解析核心
│   │   └── usmart_interface.c         # 接口桥接层
│   └── LOGGER/                        # 日志系统
│       └── logger.c
│
├── Drivers/BSP/                       # 板级支持包（硬件驱动）
│   ├── EMM_V5/                        # 张大头电机驱动
│   │   ├── emm_v5.c                   # 协议封装（15个API）
│   │   ├── emm_uart.c                 # 统一RS485通信层
│   │   └── emm_state.c                # 状态机（可选）
│   ├── LED/                           # LED驱动
│   ├── KEY/                           # 按键驱动
│   ├── IWDG/                          # 看门狗驱动
│   └── PROTOCOL_ROUTER/               # 协议路由器（Modbus+Emm_V5共存）
│       ├── protocol_router.c
│       └── protocol_router.h
│
└── Drivers/SYSTEM/                    # 系统基础设施
    ├── sys/                           # 系统配置（时钟、中断）
    ├── delay/                         # 延时函数
    ├── usart/                         # 串口驱动
    └── fifo/                          # FIFO缓冲区
```

---

## ✅ 正确示例

### 示例1: 多电机管理器（V3.1修正后）

```c
// ❌ 错误位置：Core/App/multi_motor_manager.c
// ✅ 正确位置：Drivers/Middlewares/MULTI_MOTOR/multi_motor_manager.c

/**
 * @file    multi_motor_manager.c
 * @author  STM32_485 Project (Middlewares Layer)  // 明确标注层级
 * @brief   多电机管理中间件实现
 * 
 * 架构定位：
 * - 向上提供：标准化API（供App层调用）
 * - 向下依赖：BSP/EMM_V5层
 * - 可移植：不依赖具体App业务
 */

// App层调用（motor_zdt.c）：
void motor_control_task(void) {
    if (key == KEY0_PRES) {
        // ✅ App层只做业务编排
        multi_motor_enable_batch(0x000F, true);  // 调用Middlewares层
    }
}
```

### 示例2: Modbus任务调度

```c
// ✅ 正确：Core/App/modbus_task.c（业务编排）
void modbus_task_run(void) {
    if (!g_modbus_frame_complete) return;
    
    // ✅ 调用Middlewares层协议栈
    modbus_rtu_parse_frame(g_modbus_rx_buffer, g_modbus_rx_count, &req);
    modbus_gateway_process_request(&req, &resp);
    
    // ✅ 调用SYSTEM层发送
    HAL_UART_Transmit(&g_uart2_handle, tx_buffer, tx_len, 100);
}
```

### 示例3: 电机驱动封装

```c
// ✅ 正确：Drivers/BSP/EMM_V5/emm_v5.c（硬件抽象）
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t speed, ...) {
    // ✅ 封装硬件通信协议
    uint8_t cmd[10];
    cmd[0] = 0x01;
    cmd[1] = addr;
    cmd[2] = 0xFD;
    // ...构造帧
    
    // ✅ 调用通信层发送
    emm_uart_send(cmd, cmd_len);
}
```

---

## ❌ 错误示例

### 反例1: 业务逻辑混入BSP层

```c
// ❌ 错误：在LED驱动里做状态机
// Drivers/BSP/LED/led.c
void led_task(void) {
    static uint8_t state = 0;
    
    // ❌ 状态机应该在App层！
    switch (state) {
        case 0: 
            if (motor_running) {
                led_on();
                state = 1;
            }
            break;
        // ...
    }
}

// ✅ 正确：LED驱动只提供开关接口
void led_on(void) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

// ✅ 状态机放App层（motor_zdt.c）
void motor_led_task(void) {
    if (motor_is_running()) {
        led_on();
    } else {
        led_off();
    }
}
```

### 反例2: 复杂算法放App层

```c
// ❌ 错误：PID算法在App层实现
// Core/App/main.c
void main_loop(void) {
    // ❌ 100行PID算法不应该在main.c！
    float error = target - current;
    float integral += error * dt;
    float derivative = (error - last_error) / dt;
    float output = kp*error + ki*integral + kd*derivative;
    // ...
}

// ✅ 正确：封装到Middlewares层
// Drivers/Middlewares/PID/pid_controller.c
float pid_compute(pid_t *pid, float setpoint, float input) {
    // PID算法实现
}

// ✅ App层只调用
void main_loop(void) {
    float output = pid_compute(&motor_pid, target, current);
    motor_set_output(output);
}
```

---

## 🔍 架构自检清单

在添加新功能时，问自己3个问题：

### 1. 这是业务编排还是功能实现？
- **业务编排** → `Core/App/`  
  例如：按键按下→调用电机控制函数
  
- **功能实现** → `Drivers/Middlewares/`  
  例如：PID算法、协议解析、多电机管理

### 2. 这个功能能否被其他App复用？
- **能复用** → `Drivers/Middlewares/`  
  例如：Modbus协议栈可用于其他STM32项目
  
- **特定业务** → `Core/App/`  
  例如：特定产品的按键映射逻辑

### 3. 这个模块直接操作硬件吗？
- **直接操作** → `Drivers/BSP/`  
  例如：GPIO读写、UART发送、I2C通信
  
- **不操作硬件** → `Drivers/Middlewares/` 或 `Core/App/`  
  例如：数据处理、状态管理、算法计算

---

## 📋 CMakeLists.txt规范

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ============ App层（纯业务逻辑编排） ============
    Core/App/main.c
    Core/App/motor_zdt.c
    Core/App/modbus_task.c
    # ❌ 不要在这里添加复杂实现！
    
    # ============ Drivers/Middlewares（中间件层） ============
    Drivers/Middlewares/MODBUS/modbus_rtu.c
    Drivers/Middlewares/MULTI_MOTOR/multi_motor_manager.c  # ✅ 正确位置
    Drivers/Middlewares/USMART/usmart.c
    # ✅ 功能实现放这里
    
    # ============ Drivers/BSP（板级硬件驱动） ============
    Drivers/BSP/EMM_V5/emm_v5.c
    Drivers/BSP/LED/led.c
    # ✅ 硬件封装放这里
)
```

---

## 🎓 架构培养建议

### 养成好习惯的方法：

1. **新功能前先分类**
   ```
   问：这是什么？
   答：多电机管理
   
   问：是业务编排还是功能实现？
   答：功能实现（包含算法、数据结构）
   
   问：能否被其他项目复用？
   答：能（任何需要多电机的STM32项目都能用）
   
   结论：放Middlewares层！
   ```

2. **文件头明确标注层级**
   ```c
   /**
    * @author  STM32_485 Project (Middlewares Layer)  // 强制写明
    * @brief   多电机管理中间件
    */
   ```

3. **代码审查3要素**
   - ✅ 目录位置正确
   - ✅ 依赖关系合理（不能Middlewares依赖App）
   - ✅ 职责单一（一个文件只做一件事）

4. **重构信号识别**
   - 🚨 App层文件超过200行 → 可能有实现逻辑混入
   - 🚨 BSP层出现if-else状态机 → 业务逻辑应该上移
   - 🚨 Middlewares层调用App层函数 → 依赖关系倒置

---

## 📚 参考资料

- 项目架构文档：`.github/copilot-instructions.md`
- 正点原子HAL库教程：遵循相同分层原则
- Clean Architecture（Robert C. Martin）：依赖倒置原则

---

**最后建议**：每次添加新代码前，先在纸上画出依赖关系图，确认层级正确再动手！
