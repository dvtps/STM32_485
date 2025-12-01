#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Modbus网关简化测试脚本（适配pymodbus 3.x）
"""

import sys
from pymodbus.client import ModbusSerialClient

# 配置参数
SERIAL_PORT = 'COM3'
BAUDRATE = 115200
DEVICE_ID = 1
TIMEOUT = 2.0

def main():
    print("\n" + "="*60)
    print(" STM32 Modbus网关简化测试 (pymodbus 3.x)")
    print("="*60)
    print(f"串口: {SERIAL_PORT}, 波特率: {BAUDRATE}, 从机地址: {DEVICE_ID}")
    print("="*60 + "\n")
    
    # 创建客户端
    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT,
        parity='N',
        stopbits=1,
        bytesize=8
    )
    
    # 连接
    if not client.connect():
        print("❌ 无法连接到串口")
        return 1
    
    print("✅ 已连接到", SERIAL_PORT)
    
    # 测试1: 读取固件版本（地址0x0007）
    print("\n【测试1】读取固件版本（地址0x0007）")
    try:
        result = client.read_holding_registers(address=0x0007, count=1, device_id=DEVICE_ID)
        if result.isError():
            print(f"❌ 错误: {result}")
        else:
            fw_ver = result.registers[0]
            print(f"✅ 固件版本: 0x{fw_ver:04X} (V{fw_ver>>8}.{fw_ver&0xFF})")
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 测试2: 读取硬件版本（地址0x0008）
    print("\n【测试2】读取硬件版本（地址0x0008）")
    try:
        result = client.read_holding_registers(address=0x0008, count=1, device_id=DEVICE_ID)
        if result.isError():
            print(f"❌ 错误: {result}")
        else:
            hw_ver = result.registers[0]
            print(f"✅ 硬件版本: 0x{hw_ver:04X}")
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 测试3: 写入电机速度（地址0x0103, 值10000=1000RPM）
    print("\n【测试3】写入电机1速度寄存器（地址0x0103, 值10000）")
    try:
        result = client.write_register(address=0x0103, value=10000, device_id=DEVICE_ID)
        if result.isError():
            print(f"❌ 错误: {result}")
        else:
            print(f"✅ 写入成功")
            # 回读验证
            result = client.read_holding_registers(address=0x0103, count=1, device_id=DEVICE_ID)
            if not result.isError():
                print(f"✅ 回读验证: {result.registers[0]} (应为10000)")
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 测试4: 写入线圈（地址0x0C00, 使能电机1）
    print("\n【测试4】写入线圈（地址0x0C00, 使能电机1）")
    try:
        result = client.write_coil(address=0x0C00, value=True, device_id=DEVICE_ID)
        if result.isError():
            print(f"❌ 错误: {result}")
        else:
            print(f"✅ 写入成功")
            # 回读验证
            result = client.read_coils(address=0x0C00, count=1, device_id=DEVICE_ID)
            if not result.isError():
                print(f"✅ 回读验证: {'ON' if result.bits[0] else 'OFF'}")
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 关闭连接
    client.close()
    print("\n" + "="*60)
    print("测试完成")
    print("="*60)
    
    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n程序被中断")
        sys.exit(1)
