#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Modbus网关测试脚本
功能: 验证Modbus RTU通讯、寄存器读写、电机控制
作者: STM32_485 Project
日期: 2025-12-01
"""

import time
import sys
from pymodbus.client import ModbusSerialClient

# ============= 配置参数 =============
SERIAL_PORT = 'COM7'        # 串口号（根据实际情况修改）
BAUDRATE = 115200           # 波特率
SLAVE_ADDRESS = 1           # 从机地址
TIMEOUT = 1.0               # 超时时间(秒)

# ============= Modbus寄存器地址定义 =============
# 全局控制区
REG_SYS_FIRMWARE_VERSION = 0x0007
REG_SYS_HARDWARE_VERSION = 0x0008
REG_SYS_DEVICE_ID = 0x0009

# 电机1控制区 (基地址 0x0100)
MOTOR1_BASE = 0x0100
MOTOR_REG_ENABLE = 0
MOTOR_REG_CTRL_MODE = 1
MOTOR_REG_DIRECTION = 2
MOTOR_REG_SPEED = 3
MOTOR_REG_ACCELERATION = 4
MOTOR_REG_DECELERATION = 5
MOTOR_REG_POSITION_H = 6
MOTOR_REG_POSITION_L = 7
MOTOR_REG_EXEC_COMMAND = 8
MOTOR_REG_SYNC_FLAG = 9
MOTOR_REG_MOTION_TYPE = 10
MOTOR_REG_CURRENT_LIMIT = 11

# 电机1状态区 (基地址 0x0500)
MOTOR1_STATUS_BASE = 0x0500
MOTOR_STATUS_REAL_POSITION_H = 0
MOTOR_STATUS_REAL_POSITION_L = 1
MOTOR_STATUS_REAL_SPEED = 2
MOTOR_STATUS_REAL_CURRENT = 3
MOTOR_STATUS_FLAGS = 16

# 线圈地址
COIL_MOTOR1_ENABLE = 0x0C00
COIL_EMERGENCY_STOP = 0x0C10

# 命令码
CMD_ENABLE = 1
CMD_DISABLE = 2
CMD_POS_MOVE = 3
CMD_VEL_MOVE = 4
CMD_STOP = 5
CMD_CLEAR_POS = 7

# ============= 工具函数 =============
def print_success(msg):
    print(f"✅ {msg}")

def print_error(msg):
    print(f"❌ {msg}")

def print_info(msg):
    print(f"ℹ️  {msg}")

def print_warning(msg):
    print(f"⚠️  {msg}")

def check_connection(client):
    """检查连接状态"""
    if not client.connect():
        print_error("无法连接到串口，请检查：")
        print("  1. 串口号是否正确（当前: {}）".format(SERIAL_PORT))
        print("  2. STM32是否已烧录程序并上电")
        print("  3. USB线是否正常连接")
        print("  4. 是否设置了MODBUS_GATEWAY_MODE=1")
        return False
    print_success(f"已连接到 {SERIAL_PORT}")
    return True

# ============= 测试用例 =============
def test_01_read_system_info(client):
    """测试1: 读取系统信息（功能码0x03）"""
    print("\n" + "="*60)
    print("测试1: 读取系统信息（固件版本、硬件版本、设备ID）")
    print("="*60)
    
    try:
        # 读取3个寄存器
        result = client.read_holding_registers(address=REG_SYS_FIRMWARE_VERSION, count=3, device_id=SLAVE_ADDRESS)
        
        if result.isError():
            print_error(f"读取失败: {result}")
            return False
        
        fw_ver = result.registers[0]
        hw_ver = result.registers[1]
        device_id = result.registers[2]
        
        print_success("读取成功:")
        print(f"  - 固件版本: V{fw_ver >> 8}.{fw_ver & 0xFF}")
        print(f"  - 硬件版本: 0x{hw_ver:04X}")
        print(f"  - 设备ID: {device_id}")
        
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False

def test_02_write_single_register(client):
    """测试2: 写单个寄存器（功能码0x06）"""
    print("\n" + "="*60)
    print("测试2: 写单个寄存器（设置电机1速度为1000 RPM）")
    print("="*60)
    
    try:
        # 写入速度寄存器（1000 RPM = 10000个0.1RPM）
        result = client.write_register(MOTOR1_BASE + MOTOR_REG_SPEED, 10000, address=start_addr, count=num, device_id=SLAVE_ADDRESS)
        
        if result.isError():
            print_error(f"写入失败: {result}")
            return False
        
        print_success("写入成功: SPEED = 10000 (1000.0 RPM)")
        
        # 回读验证
        result = client.read_holding_registers(MOTOR1_BASE + MOTOR_REG_SPEED, 1, address=start_addr, count=num, device_id=SLAVE_ADDRESS)
        if not result.isError():
            speed = result.registers[0]
            print_info(f"回读验证: SPEED = {speed} ({speed/10.0} RPM)")
        
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False

def test_03_write_multiple_registers(client):
    """测试3: 写多个寄存器（功能码0x10）"""
    print("\n" + "="*60)
    print("测试3: 批量写入电机参数（模式+速度+加速度+位置）")
    print("="*60)
    
    try:
        # 批量写入: 控制模式=梯形曲线, 方向=顺时针, 速度=500RPM, 加速度=100
        values = [
            3,      # CTRL_MODE = 梯形曲线
            0,      # DIRECTION = 顺时针
            5000,   # SPEED = 500.0 RPM
            100,    # ACCELERATION
            100,    # DECELERATION
            0,      # POSITION_H = 0
            3600,   # POSITION_L = 360.0° (1圈)
        ]
        
        result = client.write_registers(
            MOTOR1_BASE + MOTOR_REG_CTRL_MODE, 
            values, 
            address=start_addr, count=num, device_id=SLAVE_ADDRESS
        )
        
        if result.isError():
            print_error(f"写入失败: {result}")
            return False
        
        print_success("批量写入成功:")
        print(f"  - 控制模式: 3 (梯形曲线)")
        print(f"  - 方向: 0 (顺时针)")
        print(f"  - 速度: 5000 (500.0 RPM)")
        print(f"  - 加速度: 100")
        print(f"  - 位置: 3600 (360.0°)")
        
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False

def test_04_single_motor_position_control(client):
    """测试4: 单电机位置控制"""
    print("\n" + "="*60)
    print("测试4: 单电机位置控制（执行1圈运动）")
    print("="*60)
    
    try:
        # 步骤1: 使能电机
        print_info("步骤1: 使能电机...")
        result = client.write_register(
            MOTOR1_BASE + MOTOR_REG_ENABLE, 
            1, 
            address=start_addr, count=num, device_id=SLAVE_ADDRESS
        )
        if result.isError():
            print_error("使能失败")
            return False
        print_success("电机已使能")
        time.sleep(0.1)
        
        # 步骤2: 配置运动参数（已在测试3完成）
        print_info("步骤2: 运动参数已配置（500 RPM, 360°）")
        
        # 步骤3: 执行位置运动命令
        print_info("步骤3: 执行位置运动命令...")
        result = client.write_register(
            MOTOR1_BASE + MOTOR_REG_EXEC_COMMAND, 
            CMD_POS_MOVE, 
            address=start_addr, count=num, device_id=SLAVE_ADDRESS
        )
        if result.isError():
            print_error("执行失败")
            return False
        
        print_success("位置运动命令已发送")
        print_info("电机应正在运动（请观察实际电机或LED指示）...")
        
        # 等待运动完成（假设需要5秒）
        print_info("等待5秒...")
        time.sleep(5)
        
        print_success("测试完成")
        
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False

def test_05_read_motor_status(client):
    """测试5: 读取电机状态（功能码0x04）"""
    print("\n" + "="*60)
    print("测试5: 读取电机状态寄存器（实时位置、速度、电流）")
    print("="*60)
    
    try:
        # 读取电机状态（前6个寄存器）
        result = client.read_input_registers(
            MOTOR1_STATUS_BASE, 
            6, 
            address=start_addr, count=num, device_id=SLAVE_ADDRESS
        )
        
        if result.isError():
            print_error(f"读取失败: {result}")
            return False
        
        # 解析状态
        pos_h = result.registers[0]
        pos_l = result.registers[1]
        speed = result.registers[2]
        current = result.registers[3]
        voltage = result.registers[4]
        temp = result.registers[5]
        
        # 合并32位位置（有符号）
        position = (pos_h << 16) | pos_l
        if position & 0x80000000:  # 负数
            position -= 0x100000000
        
        print_success("读取成功:")
        print(f"  - 实时位置: {position/10.0:.1f}° ({position}个0.1°)")
        print(f"  - 实时速度: {speed/10.0:.1f} RPM")
        print(f"  - 实时电流: {current} mA")
        print(f"  - 实时电压: {voltage} mV")
        print(f"  - 实时温度: {temp}℃")
        
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False

def test_06_coil_operations(client):
    """测试6: 线圈操作（快速使能/失能）"""
    print("\n" + "="*60)
    print("测试6: 线圈操作（功能码0x05 - 快速使能控制）")
    print("="*60)
    
    try:
        # 写线圈：使能电机1
        print_info("使能电机1...")
        result = client.write_coil(COIL_MOTOR1_ENABLE, True, address=start_addr, count=num, device_id=SLAVE_ADDRESS)
        if result.isError():
            print_error("写入失败")
            return False
        print_success("电机1已使能（通过线圈）")
        
        time.sleep(1)
        
        # 读取线圈状态
        result = client.read_coils(COIL_MOTOR1_ENABLE, 1, address=start_addr, count=num, device_id=SLAVE_ADDRESS)
        if not result.isError():
            status = result.bits[0]
            print_info(f"线圈状态: {'ON' if status else 'OFF'}")
        
        # 失能电机1
        print_info("失能电机1...")
        result = client.write_coil(COIL_MOTOR1_ENABLE, False, address=start_addr, count=num, device_id=SLAVE_ADDRESS)
        if result.isError():
            print_error("写入失败")
            return False
        print_success("电机1已失能")
        
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False

def test_07_emergency_stop(client):
    """测试7: 紧急停止（所有电机）"""
    print("\n" + "="*60)
    print("测试7: 紧急停止功能（触发全局急停线圈）")
    print("="*60)
    
    try:
        print_warning("触发紧急停止...")
        result = client.write_coil(COIL_EMERGENCY_STOP, True, address=start_addr, count=num, device_id=SLAVE_ADDRESS)
        
        if result.isError():
            print_error("触发失败")
            return False
        
        print_success("紧急停止已触发（所有电机应立即停止）")
        
        # 复位急停线圈
        time.sleep(0.5)
        client.write_coil(COIL_EMERGENCY_STOP, False, address=start_addr, count=num, device_id=SLAVE_ADDRESS)
        print_info("紧急停止线圈已复位")
        
        return True
        
    except Exception as e:
        print_error(f"异常: {e}")
        return False

# ============= 主程序 =============
def main():
    print("\n" + "="*60)
    print(" STM32 Modbus网关测试程序 V1.0")
    print("="*60)
    print(f"串口: {SERIAL_PORT}")
    print(f"波特率: {BAUDRATE}")
    print(f"从机地址: {SLAVE_ADDRESS}")
    print(f"超时时间: {TIMEOUT}s")
    print("="*60)
    
    # 创建Modbus客户端
    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT,
        parity='N',
        stopbits=1,
        bytesize=8
    )
    
    # 检查连接
    if not check_connection(client):
        return 1
    
    # 执行测试
    tests = [
        ("读取系统信息", test_01_read_system_info),
        ("写单个寄存器", test_02_write_single_register),
        ("写多个寄存器", test_03_write_multiple_registers),
        ("单电机位置控制", test_04_single_motor_position_control),
        ("读取电机状态", test_05_read_motor_status),
        ("线圈操作", test_06_coil_operations),
        ("紧急停止", test_07_emergency_stop),
    ]
    
    passed = 0
    failed = 0
    
    for i, (name, test_func) in enumerate(tests, 1):
        print(f"\n>>> 运行测试 {i}/{len(tests)}: {name}")
        try:
            if test_func(client):
                passed += 1
            else:
                failed += 1
                print_error(f"测试失败: {name}")
        except KeyboardInterrupt:
            print("\n\n中断测试...")
            break
        except Exception as e:
            failed += 1
            print_error(f"测试异常: {name} - {e}")
        
        time.sleep(0.5)  # 测试间隔
    
    # 关闭连接
    client.close()
    
    # 输出测试结果
    print("\n" + "="*60)
    print(" 测试结果汇总")
    print("="*60)
    print(f"总计: {passed + failed} 个测试")
    print(f"✅ 通过: {passed}")
    print(f"❌ 失败: {failed}")
    print("="*60)
    
    return 0 if failed == 0 else 1

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n程序被中断")
        sys.exit(1)
