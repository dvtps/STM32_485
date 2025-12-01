# STM32_485 架构重构日志 V3.1

## 重构时间
2025-12-01

## 问题背景

用户发现AI在实现功能扩展时，习惯性地将中间件功能直接放在 `Core/App/` 目录，违反了分层架构原则：

**典型问题**：
1. `multi_motor_manager.c` 被放在 `Core/App/`（应在 `Drivers/Middlewares/MULTI_MOTOR/`）
2. `modbus_task.c` 包含协议解析逻辑（应调用 Middlewares 层 API）

**架构检查器报告**：
```
❌ 错误 (ERROR):
  [SIZE] D:/STM32/Projects/ZDT/STM32_485/Core/App/modbus_task.c
    App层文件超过200行 (202 行 > 200 行限制)
```

---

## 改进目标

**核心原则**：App层只做编排，不做实现

- **App层**：业务流程编排，调用下层API（<200行限制）
- **Middlewares层**：可复用功能实现（协议栈、多电机管理）
- **BSP层**：硬件驱动封装（电机通信、LED/KEY）
- **SYSTEM层**：底层基础设施（USART、延时、时钟）

---

## 改进项目1: modbus_task.c 架构重构

### 问题分析

**重复代码**：
- `modbus_task.c:62-140` 包含 78 行静态函数 `modbus_process_request()`
- 该函数使用 switch-case 处理 Modbus 功能码 (0x03/0x06/0x10/0x04)
- 完全相同的逻辑已存在于 `modbus_rtu.c:255` 的 `modbus_rtu_process_request()`

**架构违例**：
- App层不应包含协议解析细节
- 重复实现导致维护困难（两处代码需同步修改）

### 重构步骤

#### 步骤1: 删除重复函数（-78行）

```c
// 删除前 (Core/App/modbus_task.c:62-140)
static int modbus_process_request(const modbus_rtu_frame_t *req, modbus_rtu_frame_t *resp)
{
    // ...78行switch-case协议处理逻辑
}
```

#### 步骤2: 删除无用变量（-2行）

```c
// 删除前
static modbus_rtu_frame_t g_modbus_req_frame = {0};   /* 请求帧 */
static modbus_rtu_frame_t g_modbus_resp_frame = {0};  /* 响应帧 */
```

#### 步骤3: 简化主循环函数（-33行）

```c
// 重构后 (Core/App/modbus_task.c:63-88)
void modbus_task_run(void)
{
    uint16_t tx_len = 0;
    
    if (!g_modbus_frame_complete) {
        return;
    }
    
    g_modbus_frame_complete = 0;
    
    /* V3.1优化: 直接调用Middlewares层API */
    int result = modbus_rtu_process_request(
        g_modbus_rx_buffer,     /* 输入：接收缓冲区 */
        g_modbus_rx_count,      /* 输入：接收长度 */
        g_modbus_tx_buffer,     /* 输出：发送缓冲区 */
        &tx_len                 /* 输出：发送长度 */
    );
    
    if (result == 0 && tx_len > 0) {
        HAL_UART_Transmit(&g_uart2_handle, g_modbus_tx_buffer, tx_len, 100);
    }
}
```

**关键改进**：
- 移除解析帧结构体操作（原 `modbus_rtu_parse_frame()` 调用）
- 移除地址匹配检查逻辑（由 `modbus_rtu_process_request` 处理）
- 移除手动构造响应帧代码（由 `modbus_rtu_process_request` 处理）
- App层只负责缓冲区管理和UART发送调度

### 重构成果

#### 代码行数优化

| 文件 | 重构前 | 重构后 | 减少 | 优化率 |
|------|--------|--------|------|--------|
| modbus_task.c | 203 行 | **90 行** | -113 行 | **-55.7%** |

#### 资源占用变化

| 资源 | 重构前 | 重构后 | 变化 | 说明 |
|------|--------|--------|------|------|
| Flash | 37456 B | 39792 B | +2336 B | 调用完整协议处理函数，增加少量代码 |
| RAM | 11312 B | 10784 B | **-528 B** | 删除 2 个 modbus_rtu_frame_t 结构体 |

#### 架构合规性

```bash
$ python check_architecture.py
============================================================
检查完成: 0 个错误, 6 个警告
============================================================
```

✅ **改进项目1完成**：modbus_task.c 符合 App层 200 行限制

---

## 架构改进总结

### 正确的分层调用模式

```
┌─────────────────────────────────────────────────┐
│ Core/App/modbus_task.c (90行)                   │
│ ┌─────────────────────────────────────────────┐ │
│ │ void modbus_task_run(void)                  │ │
│ │ {                                            │ │
│ │     // 1. 检查缓冲区                         │ │
│ │     if (!g_modbus_frame_complete) return;   │ │
│ │                                              │ │
│ │     // 2. 调用Middlewares层API ⬇️            │ │
│ └──────────────┬──────────────────────────────┘ │
└────────────────┼────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│ Drivers/Middlewares/MODBUS/modbus_rtu.c (727行) │
│ ┌─────────────────────────────────────────────┐ │
│ │ int modbus_rtu_process_request(...)         │ │
│ │ {                                            │ │
│ │     // 1. 帧解析 (CRC校验/地址匹配)          │ │
│ │     // 2. 功能码分发 (switch-case)           │ │
│ │     // 3. 调用网关层 ⬇️                       │ │
│ └──────────────┬──────────────────────────────┘ │
└────────────────┼────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│ Drivers/Middlewares/MODBUS/modbus_gateway.c     │
│ ┌─────────────────────────────────────────────┐ │
│ │ modbus_gateway_read_holding_registers(...)  │ │
│ │ {                                            │ │
│ │     // 寄存器映射到电机控制参数              │ │
│ │     // 调用BSP层API ⬇️                       │ │
│ └──────────────┬──────────────────────────────┘ │
└────────────────┼────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│ Drivers/BSP/EMM_V5/emm_v5.c (379行)             │
│ ┌─────────────────────────────────────────────┐ │
│ │ Emm_V5_Pos_Control(...)                     │ │
│ │ {                                            │ │
│ │     // 构造电机协议帧                        │ │
│ │     // 通过USART2发送                        │ │
│ └─────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────┘
```

### AI开发习惯培养

#### ❌ 反面模式（V3.0之前）

```c
// Core/App/modbus_task.c (错误示例)
void modbus_task_run(void)
{
    // ❌ 在App层实现协议细节
    switch (req->function_code) {
        case MODBUS_FC_READ_HOLDING_REGS:
            start_addr = (req->data[0] << 8) | req->data[1];
            num_regs = (req->data[2] << 8) | req->data[3];
            // ...60行协议处理代码
            break;
    }
    
    // ❌ 手动构造响应帧
    tx_buffer[0] = slave_addr;
    tx_buffer[1] = function_code;
    memcpy(&tx_buffer[2], data, data_len);
    // ...
}
```

#### ✅ 正确模式（V3.1架构）

```c
// Core/App/modbus_task.c (正确示例)
void modbus_task_run(void)
{
    // ✅ App层仅做编排
    if (!g_modbus_frame_complete) return;
    g_modbus_frame_complete = 0;
    
    // ✅ 调用Middlewares层统一接口
    int result = modbus_rtu_process_request(
        g_modbus_rx_buffer,
        g_modbus_rx_count,
        g_modbus_tx_buffer,
        &tx_len
    );
    
    // ✅ 仅负责最终发送
    if (result == 0 && tx_len > 0) {
        HAL_UART_Transmit(&g_uart2_handle, g_modbus_tx_buffer, tx_len, 100);
    }
}
```

### 架构规则速查卡

| 层级 | 职责 | 行数限制 | 可调用 | 禁止调用 |
|------|------|----------|--------|----------|
| **App** | 业务编排 | <200 行 | Middlewares/BSP | HAL_*/LL_* |
| **Middlewares** | 功能实现 | <500 行 | BSP/SYSTEM | 硬件寄存器 |
| **BSP** | 硬件抽象 | <300 行 | HAL_*/SYSTEM | - |
| **SYSTEM** | 基础设施 | <500 行 | HAL_* | - |

### 自检清单

开发新功能前，AI应自问：

1. ☑️ 这个函数属于哪一层？
   - 业务逻辑 → App
   - 协议/算法实现 → Middlewares
   - 硬件驱动 → BSP
   - 底层基础 → SYSTEM

2. ☑️ 是否有现成的下层API可用？
   - 先搜索现有函数（`grep_search`）
   - 避免重复实现

3. ☑️ 文件行数是否超限？
   - App < 200 行
   - BSP < 300 行
   - Middlewares/SYSTEM < 500 行

4. ☑️ 是否直接调用HAL库？
   - App层 → 禁止
   - Middlewares层 → 尽量通过BSP封装
   - BSP层 → 允许

---

## 后续改进计划

### 改进项目2: usmart.c 拆分（可选）

**当前状态**: 523 行（超限 23 行）

**拆分方案**:
- `usmart_parser.c`: 命令解析 (~200行)
- `usmart_executor.c`: 函数执行 (~200行)
- `usmart_interface.c`: 对外接口 (~100行)

### 改进项目3: modbus_rtu.c 拆分（可选）

**当前状态**: 727 行（超限 227 行）

**拆分方案**:
- `modbus_rtu_frame.c`: 帧构造/解析
- `modbus_rtu_handler.c`: 功能码处理
- `modbus_rtu_crc.c`: CRC计算

---

## 版本历史

- **V3.1** (2025-12-01): 
  - ✅ 重构 modbus_task.c (203→90行)
  - ✅ 架构检查器通过 (0个错误)
  - ✅ 培养AI分层架构习惯

- **V3.0** (2025-11-30):
  - 新增协议路由器 (Modbus+Emm_V5共存)
  - 新增多电机管理中间件
  - 创建架构检查工具

- **V2.0** (2025-11-30):
  - HAL库移植
  - 目录重组

---

## 参考文档

- `Docs/ARCHITECTURE_GUIDE.md`: 完整架构规范
- `Docs/check_architecture.py`: 自动化检查工具
- `.github/copilot-instructions.md`: AI开发指南（已同步更新）

---

*此文档记录架构优化过程，供后续开发参考*
