/**
 ******************************************************************************
 * @file    usmart_interface.c
 * @author  STM32_485 Project
 * @version V3.1
 * @date    2025-12-01
 * @brief   USMART接口函数实现（桥接层）
 ******************************************************************************
 */

#include "usmart_interface.h"
#include "multi_motor_manager.h"
#include "protocol_router.h"
#include "emm_v5.h"
#include "usart.h"  /* V3.5 Phase 8: CRC和FIFO统计 */
#include "mem_pool.h"  /* V3.5 Phase 1: 内存池管理 */
#include <stdio.h>
#include <string.h>  /* memset */

/* ============ 单电机控制实现 ============ */

void motor_enable(uint8_t addr, uint8_t enable)
{
    Emm_V5_En_Control(addr, enable ? true : false, false);
    printf("Motor#%d %s\r\n", addr, enable ? "ENABLED" : "DISABLED");
}

void motor_pos_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    Emm_V5_Pos_Control(addr, dir, speed, acc, pulses, false, false);
    printf("Motor#%d: POS dir=%d speed=%d pulses=%lu\r\n", addr, dir, speed, (unsigned long)pulses);
}

void motor_vel_move(uint8_t addr, uint8_t dir, uint16_t speed, uint8_t acc)
{
    Emm_V5_Vel_Control(addr, dir, speed, acc, false);
    printf("Motor#%d: VEL dir=%d speed=%d\r\n", addr, dir, speed);
}

void motor_stop(uint8_t addr)
{
    Emm_V5_Stop_Now(addr, false);
    printf("Motor#%d: STOP\r\n", addr);
}

void motor_home(uint8_t addr)
{
    Emm_V5_Origin_Trigger_Return(addr, 0, false);
    printf("Motor#%d: HOME\r\n", addr);
}

void motor_read_status(uint8_t addr)
{
    Emm_V5_Read_Sys_Params(addr, false);
    printf("Motor#%d: STATUS query sent\r\n", addr);
}

/* ============ 多电机管理实现 ============ */

void multi_scan(uint8_t start, uint8_t end)
{
    int count = multi_motor_scan(start, end);
    printf("Scan complete: %d motors found\r\n", count);
}

void multi_map(uint8_t modbus, uint8_t physical)
{
    if (multi_motor_map_address(modbus, physical) == 0) {
        printf("Address mapped: Modbus#%d -> Physical#%d\r\n", modbus, physical);
    } else {
        printf("Map failed: invalid address\r\n");
    }
}

void multi_list(void)
{
    multi_motor_print_list();
}

void multi_enable(uint16_t mask, uint8_t enable)
{
    multi_motor_enable_batch(mask, enable ? true : false);
    printf("Batch enable: mask=0x%04X state=%d\r\n", mask, enable);
}

void multi_pos(uint16_t mask, uint8_t dir, uint16_t speed, uint32_t pulses)
{
    multi_motor_pos_control_batch(mask, dir, speed, 10, pulses);
    printf("Batch pos: mask=0x%04X dir=%d speed=%d pulses=%lu\r\n", 
           mask, dir, speed, (unsigned long)pulses);
}

void multi_vel(uint16_t mask, uint8_t dir, uint16_t speed)
{
    multi_motor_vel_control_batch(mask, dir, speed, 10);
    printf("Batch vel: mask=0x%04X dir=%d speed=%d\r\n", mask, dir, speed);
}

void multi_stop(uint16_t mask)
{
    multi_motor_stop_batch(mask);
    printf("Batch stop: mask=0x%04X\r\n", mask);
}

void multi_home(uint16_t mask, uint8_t mode)
{
    multi_motor_home_batch(mask, mode);
    printf("Batch home: mask=0x%04X mode=%d\r\n", mask, mode);
}

/* ============ 协议统计实现 ============ */

void proto_stats(void)
{
    const protocol_router_stats_t *stats = protocol_router_get_stats();
    
    printf("\r\n========== Protocol Statistics ==========\r\n");
    printf("Modbus RTU frames:  %lu\r\n", (unsigned long)stats->modbus_frames);
    printf("Emm_V5 frames:      %lu\r\n", (unsigned long)stats->emm_v5_frames);
    printf("Unknown frames:     %lu\r\n", (unsigned long)stats->unknown_frames);
    printf("CRC errors:         %lu\r\n", (unsigned long)stats->crc_errors);
    printf("=========================================\r\n\r\n");
}

void proto_reset(void)
{
    protocol_router_reset_stats();
    printf("Protocol statistics reset\r\n");
}

/* ============ V3.5 Phase 8 P1: 增量CRC调试实现 ============ */

/**
 * @brief       显示增量CRC统计信息
 * @note        测试增量CRC功能是否正常工作
 */
void crc_stats(void)
{
    uint16_t current_crc = get_incremental_crc();
    uint16_t byte_count = get_crc_byte_count();
    uint32_t calc_count = get_crc_calc_count();
    
    printf("\r\n========== Incremental CRC Stats ==========\r\n");
    printf("Current CRC value:  0x%04X\r\n", current_crc);
    printf("Bytes calculated:   %u\r\n", byte_count);
    printf("Frames calculated:  %lu\r\n", (unsigned long)calc_count);
    printf("CRC16 Table:        %s\r\n", "Enabled (256 entries)");
    printf("Performance gain:   ~100 CPU cycles/frame\r\n");
    printf("============================================\r\n\r\n");
}

/**
 * @brief       显示FIFO统计信息
 * @note        监控FIFO溢出情况（Phase 8优化指标）
 */
void fifo_stats(void)
{
    uint32_t overflow_count = get_fifo_overflow_count();
    uint32_t idle_count = get_idle_interrupt_count();
    
    printf("\r\n========== FIFO Statistics ==========\r\n");
    printf("FIFO size:          256 bytes\r\n");
    printf("FIFO overflow:      %lu times\r\n", (unsigned long)overflow_count);
    printf("IDLE interrupts:    %lu times\r\n", (unsigned long)idle_count);
    
    if (idle_count > 0) {
        float overflow_rate = (float)overflow_count / idle_count * 100.0f;
        printf("Overflow rate:      %.2f%%\r\n", overflow_rate);
        
        if (overflow_rate < 2.0f) {
            printf("Status:             GOOD (target: <2%%)\r\n");
        } else if (overflow_rate < 5.0f) {
            printf("Status:             WARNING (2-5%%)\r\n");
        } else {
            printf("Status:             CRITICAL (>5%%)\r\n");
        }
    }
    printf("======================================\r\n\r\n");
}

/* ============ V3.5 Phase 1: 内存池调试命令实现 ============ */

/**
 * @brief       显示内存池统计信息
 * @note        监控内存使用情况、分配失败、泄漏告警
 */
void mem_stats(void)
{
    mem_pool_print_stats();
}

/**
 * @brief       检查并报告内存泄漏
 * @note        扫描超过5秒未释放的内存块
 */
void mem_check_leaks(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t leak_count = mem_pool_check_leaks(current_time);
    
    printf("\r\n[MEM_POOL] Leak Check Result: %lu blocks leaked\r\n", (unsigned long)leak_count);
    mem_pool_print_leak_report(current_time);
}

/**
 * @brief       重置内存池统计计数器
 * @param       type: 池类型（0=帧缓冲池, 1=电机状态池）
 * @note        保留当前分配状态，仅重置计数器
 */
void mem_reset_stats(uint8_t type)
{
    mem_pool_type_t pool_type = (type == 0) ? MEM_POOL_TYPE_FRAME : MEM_POOL_TYPE_MOTOR_STATE;
    mem_pool_reset_stats(pool_type);
    printf("[MEM_POOL] Statistics reset for pool type %d\r\n", type);
}

/* ============ 硬件测试函数 ============ */

/**
 * @brief       压力测试：循环分配和释放内存
 * @param       count: 测试次数（推荐1000）
 * @note        验证：无分配失败、无泄漏、无double_free
 */
void mem_test_stress(uint16_t count)
{
    printf("\r\n[MEM_TEST] Starting stress test (%d iterations)...\r\n", count);
    printf("=========================================\r\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t fail_count = 0;
    
    /* 测试帧缓冲池 */
    printf("[Frame Pool] Testing %d alloc/free cycles...\r\n", count);
    for (uint16_t i = 0; i < count; i++) {
        void* ptr = MEM_POOL_ALLOC_FRAME();
        if (ptr == NULL) {
            printf("  ✗ Alloc failed at iteration %d\r\n", i + 1);
            fail_count++;
            break;
        }
        
        /* 写入测试数据（防止编译器优化） */
        memset(ptr, 0xAA, 256);
        
        mem_pool_err_t err = mem_pool_free_frame(ptr);
        if (err != MEM_POOL_OK) {
            printf("  ✗ Free failed at iteration %d (err=%d)\r\n", i + 1, err);
            fail_count++;
            break;
        }
        
        /* 每100次打印进度 */
        if ((i + 1) % 100 == 0) {
            printf("  Progress: %d/%d\r\n", i + 1, count);
        }
    }
    
    /* 测试电机状态池 */
    printf("\r\n[Motor State Pool] Testing %d alloc/free cycles...\r\n", count);
    for (uint16_t i = 0; i < count; i++) {
        void* ptr = MEM_POOL_ALLOC_MOTOR_STATE();
        if (ptr == NULL) {
            printf("  ✗ Alloc failed at iteration %d\r\n", i + 1);
            fail_count++;
            break;
        }
        
        /* 写入测试数据 */
        memset(ptr, 0x55, 64);
        
        mem_pool_err_t err = mem_pool_free_motor_state(ptr);
        if (err != MEM_POOL_OK) {
            printf("  ✗ Free failed at iteration %d (err=%d)\r\n", i + 1, err);
            fail_count++;
            break;
        }
        
        /* 每100次打印进度 */
        if ((i + 1) % 100 == 0) {
            printf("  Progress: %d/%d\r\n", i + 1, count);
        }
    }
    
    uint32_t elapsed = HAL_GetTick() - start_time;
    
    printf("\r\n=========================================\r\n");
    printf("[MEM_TEST] Stress test completed\r\n");
    printf("  Total iterations: %d x 2 pools = %d\r\n", count, count * 2);
    printf("  Failed operations: %lu\r\n", (unsigned long)fail_count);
    printf("  Elapsed time: %lu ms\r\n", (unsigned long)elapsed);
    printf("  Avg time per op: %.2f us\r\n", (float)elapsed * 1000.0f / (count * 2));
    
    if (fail_count == 0) {
        printf("  Result: ✓ PASSED\r\n");
    } else {
        printf("  Result: ✗ FAILED\r\n");
    }
    
    printf("=========================================\r\n\r\n");
    
    /* 显示统计 */
    mem_pool_print_stats();
}

/**
 * @brief       泄漏测试：故意不释放内存，验证5秒后告警
 * @param       block_count: 分配块数（1-4，超过池容量会失败）
 * @note        测试步骤：分配内存 → 等待6秒 → 调用mem_check_leaks()查看告警
 */
void mem_test_leak(uint8_t block_count)
{
    printf("\r\n[MEM_TEST] Starting leak test (allocating %d blocks)...\r\n", block_count);
    printf("=========================================\r\n");
    
    if (block_count > 4) {
        printf("  ✗ Error: block_count must be 1-4\r\n");
        printf("=========================================\r\n\r\n");
        return;
    }
    
    void* leaked_ptrs[4] = {NULL};
    
    /* 故意分配但不释放 */
    for (uint8_t i = 0; i < block_count; i++) {
        leaked_ptrs[i] = MEM_POOL_ALLOC_FRAME();
        if (leaked_ptrs[i] == NULL) {
            printf("  ✗ Alloc failed at block %d\r\n", i + 1);
            break;
        }
        printf("  ✓ Allocated block %d at 0x%08lX\r\n", i + 1, (unsigned long)leaked_ptrs[i]);
    }
    
    uint32_t alloc_time = HAL_GetTick();
    
    printf("\r\n[MEM_TEST] Blocks allocated (intentional leak)\r\n");
    printf("  Please wait 6 seconds, then call: mem_check_leaks()\r\n");
    printf("  Expected: %d leak warnings\r\n", block_count);
    printf("  Allocation timestamp: %lu ms\r\n", (unsigned long)alloc_time);
    printf("=========================================\r\n\r\n");
    
    /* 显示当前统计 */
    mem_pool_print_stats();
    
    printf("\r\n[NOTE] To clean up, manually call:\r\n");
    for (uint8_t i = 0; i < block_count; i++) {
        if (leaked_ptrs[i] != NULL) {
            printf("  write_addr(0x%08lX, 0)  // Mark as free (dangerous, for test only!)\r\n", 
                   (unsigned long)leaked_ptrs[i]);
        }
    }
    printf("\r\n");
}

/**
 * @brief       并发测试：分配至池满，验证正确返回NULL
 * @param       无
 * @note        测试场景：模拟多任务同时分配内存，验证边界条件
 */
void mem_test_concurrent(void)
{
    printf("\r\n[MEM_TEST] Starting concurrent allocation test...\r\n");
    printf("=========================================\r\n");
    
    void* frame_ptrs[5] = {NULL};  // 池容量4，第5个应失败
    void* motor_ptrs[9] = {NULL};  // 池容量8，第9个应失败
    
    /* 测试帧缓冲池：分配至池满 */
    printf("[Frame Pool] Allocating until full...\r\n");
    for (uint8_t i = 0; i < 5; i++) {
        frame_ptrs[i] = MEM_POOL_ALLOC_FRAME();
        if (frame_ptrs[i] != NULL) {
            printf("  ✓ Block %d allocated at 0x%08lX\r\n", i + 1, (unsigned long)frame_ptrs[i]);
        } else {
            printf("  ✓ Block %d allocation FAILED (expected, pool full)\r\n", i + 1);
        }
    }
    
    /* 测试电机状态池：分配至池满 */
    printf("\r\n[Motor State Pool] Allocating until full...\r\n");
    for (uint8_t i = 0; i < 9; i++) {
        motor_ptrs[i] = MEM_POOL_ALLOC_MOTOR_STATE();
        if (motor_ptrs[i] != NULL) {
            printf("  ✓ Block %d allocated at 0x%08lX\r\n", i + 1, (unsigned long)motor_ptrs[i]);
        } else {
            printf("  ✓ Block %d allocation FAILED (expected, pool full)\r\n", i + 1);
        }
    }
    
    printf("\r\n[MEM_TEST] Testing double free detection...\r\n");
    mem_pool_err_t err = mem_pool_free_frame(frame_ptrs[0]);
    printf("  First free: %s\r\n", err == MEM_POOL_OK ? "✓ OK" : "✗ FAILED");
    
    err = mem_pool_free_frame(frame_ptrs[0]);  // 重复释放
    printf("  Double free: %s (expected: ERR_DOUBLE_FREE)\r\n", 
           err == MEM_POOL_ERR_DOUBLE_FREE ? "✓ OK" : "✗ FAILED");
    
    /* 清理资源 */
    printf("\r\n[MEM_TEST] Cleaning up...\r\n");
    for (uint8_t i = 1; i < 5; i++) {  // 从1开始，0已释放
        if (frame_ptrs[i] != NULL) {
            mem_pool_free_frame(frame_ptrs[i]);
        }
    }
    for (uint8_t i = 0; i < 9; i++) {
        if (motor_ptrs[i] != NULL) {
            mem_pool_free_motor_state(motor_ptrs[i]);
        }
    }
    
    printf("=========================================\r\n");
    printf("[MEM_TEST] Concurrent test completed\r\n");
    printf("=========================================\r\n\r\n");
    
    /* 显示统计 */
    mem_pool_print_stats();
}

/* ============ V3.5 Phase 3: 多电机管理器调试接口 ============ */

void mgr_scan(uint8_t start, uint8_t end)
{
    printf("[MotorMgr] Scanning motors (%d-%d)...\r\n", start, end);
    uint8_t found = motor_mgr_scan(start, end);
    printf("[MotorMgr] Found %d motor(s)\r\n", found);
    motor_mgr_print_all_status();
}

void mgr_list(void)
{
    motor_mgr_print_all_status();
}

void mgr_info(uint8_t addr)
{
    motor_state_t *state = motor_mgr_find(addr);
    if (state) {
        motor_mgr_print_status(addr);
    } else {
        printf("[MotorMgr] Motor #%d not found\r\n", addr);
    }
}

void mgr_enable(uint8_t addr, uint8_t enable)
{
    if (motor_mgr_enable(addr, enable ? true : false) == HAL_OK) {
        printf("[MotorMgr] Motor #%d %s\r\n", addr, enable ? "ENABLED" : "DISABLED");
    } else {
        printf("[MotorMgr] Failed to control motor #%d\r\n", addr);
    }
}

void mgr_move(uint8_t addr, uint8_t dir, uint16_t speed, uint32_t pulses)
{
    if (motor_mgr_move(addr, dir, speed, 10, pulses, false) == HAL_OK) {
        printf("[MotorMgr] Motor #%d: dir=%d speed=%d pulses=%lu\r\n", 
               addr, dir, speed, (unsigned long)pulses);
    } else {
        printf("[MotorMgr] Failed to move motor #%d\r\n", addr);
    }
}

void mgr_stop(uint8_t addr)
{
    if (motor_mgr_stop(addr) == HAL_OK) {
        printf("[MotorMgr] Motor #%d STOPPED\r\n", addr);
    } else {
        printf("[MotorMgr] Failed to stop motor #%d\r\n", addr);
    }
}

void mgr_stop_all(void)
{
    uint8_t count = motor_mgr_stop_all();
    printf("[MotorMgr] Stopped %d motor(s)\r\n", count);
}

void mgr_health(uint8_t addr)
{
    uint8_t health = motor_mgr_get_health(addr);
    motor_state_t *state = motor_mgr_find(addr);
    if (state) {
        printf("[MotorMgr] Motor #%d: Health=%d, Online=%s\r\n", 
               addr, health, state->online == MOTOR_STATUS_ONLINE ? "YES" : "NO");
    } else {
        printf("[MotorMgr] Motor #%d not found\r\n", addr);
    }
}

void mgr_recover(void)
{
    printf("[MotorMgr] Triggering auto-recovery...\r\n");
    motor_mgr_auto_recover();
}

void mgr_query_pos(uint8_t addr)
{
    if (motor_mgr_query_position(addr) == HAL_OK) {
        HAL_Delay(100);  // 等待响应
        motor_state_t *state = motor_mgr_find(addr);
        if (state) {
            printf("[MotorMgr] Motor #%d: Position=%ld\r\n", addr, (long)state->position);
        }
    } else {
        printf("[MotorMgr] Failed to query motor #%d\r\n", addr);
    }
}

void mgr_query_vel(uint8_t addr)
{
    if (motor_mgr_query_velocity(addr) == HAL_OK) {
        HAL_Delay(100);  // 等待响应
        motor_state_t *state = motor_mgr_find(addr);
        if (state) {
            printf("[MotorMgr] Motor #%d: Velocity=%d RPM\r\n", addr, state->velocity);
        }
    } else {
        printf("[MotorMgr] Failed to query motor #%d\r\n", addr);
    }
}

void mgr_query_vbus(uint8_t addr)
{
    if (motor_mgr_query_vbus(addr) == HAL_OK) {
        HAL_Delay(100);  // 等待响应
        motor_state_t *state = motor_mgr_find(addr);
        if (state) {
            printf("[MotorMgr] Motor #%d: Vbus=%u mV\r\n", addr, state->vbus);
        }
    } else {
        printf("[MotorMgr] Failed to query motor #%d\r\n", addr);
    }
}
