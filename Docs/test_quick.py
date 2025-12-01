#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Modbus快速验证脚本（非交互式）
直接测试指定COM口
"""

import sys
from pymodbus.client import ModbusSerialClient

# ==================== 配置区域 ====================
SERIAL_PORT = 'COM7'        # 修改为实际COM口
BAUDRATE = 115200
SLAVE_ADDRESS = 1
TIMEOUT = 2.0
# ==================================================

def test_modbus_communication():
    """快速Modbus通信测试"""
    print("\n" + "="*60)
    print(f" STM32 Modbus快速验证 - {SERIAL_PORT}")
    print("="*60)
    
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
    print(f"\n正在连接 {SERIAL_PORT} ({BAUDRATE}bps)...")
    if not client.connect():
        print(f"❌ 无法打开串口 {SERIAL_PORT}")
        print("\n可能原因:")
        print("  - 串口被其他程序占用")
        print("  - 串口号错误（请检查设备管理器）")
        print("  - USB线未连接")
        return False
    
    print(f"✅ 串口已打开")
    
    success_count = 0
    total_tests = 4
    
    # 测试1: 读取固件版本
    print("\n[测试 1/4] 读取固件版本（地址0x0007）")
    try:
        result = client.read_holding_registers(address=0x0007, count=1, device_id=SLAVE_ADDRESS)
        if result.isError():
            print(f"❌ Modbus通信失败: {result}")
            print("   可能原因: STM32未运行或FEATURE_MODBUS_ENABLE=0")
        else:
            fw_ver = result.registers[0]
            print(f"✅ 固件版本: V{fw_ver>>8}.{fw_ver&0xFF} (原始值=0x{fw_ver:04X})")
            success_count += 1
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 测试2: 读取系统信息（3个寄存器）
    print("\n[测试 2/4] 读取系统信息（地址0x0007-0x0009）")
    try:
        result = client.read_holding_registers(address=0x0007, count=3, device_id=SLAVE_ADDRESS)
        if result.isError():
            print(f"❌ 读取失败: {result}")
        else:
            fw_ver, hw_ver, dev_id = result.registers
            print(f"✅ 读取成功:")
            print(f"   固件版本: V{fw_ver>>8}.{fw_ver&0xFF}")
            print(f"   硬件版本: 0x{hw_ver:04X}")
            print(f"   设备ID: {dev_id}")
            success_count += 1
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 测试3: 写单个寄存器（电机速度）
    print("\n[测试 3/4] 写单个寄存器（地址0x0103, 值=5000）")
    try:
        result = client.write_register(address=0x0103, value=5000, device_id=SLAVE_ADDRESS)
        if result.isError():
            print(f"❌ 写入失败: {result}")
        else:
            print(f"✅ 写入成功")
            # 回读验证
            result = client.read_holding_registers(address=0x0103, count=1, device_id=SLAVE_ADDRESS)
            if not result.isError():
                value = result.registers[0]
                if value == 5000:
                    print(f"✅ 回读验证通过: {value} (500.0 RPM)")
                    success_count += 1
                else:
                    print(f"⚠️  回读验证失败: 期望5000, 实际{value}")
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 测试4: 读输入寄存器（电机状态）
    print("\n[测试 4/4] 读输入寄存器（地址0x0500-0x0505）")
    try:
        result = client.read_input_registers(address=0x0500, count=6, device_id=SLAVE_ADDRESS)
        if result.isError():
            print(f"❌ 读取失败: {result}")
        else:
            pos_h, pos_l, speed, current, voltage, temp = result.registers
            position = (pos_h << 16) | pos_l
            if position & 0x80000000:
                position -= 0x100000000
            print(f"✅ 读取成功:")
            print(f"   实时位置: {position/10.0:.1f}° ({position}脉冲)")
            print(f"   实时速度: {speed/10.0:.1f} RPM")
            print(f"   实时电流: {current} mA")
            print(f"   实时电压: {voltage} mV")
            print(f"   实时温度: {temp}℃")
            success_count += 1
    except Exception as e:
        print(f"❌ 异常: {e}")
    
    # 关闭连接
    client.close()
    
    # 结果汇总
    print("\n" + "="*60)
    print(" 测试结果汇总")
    print("="*60)
    print(f"总计: {total_tests} 个测试")
    print(f"✅ 通过: {success_count}")
    if total_tests - success_count > 0:
        print(f"❌ 失败: {total_tests - success_count}")
    print("="*60)
    
    if success_count == total_tests:
        print("\n🎉 所有测试通过！Modbus通信正常！")
        return True
    elif success_count > 0:
        print(f"\n⚠️  部分测试通过 ({success_count}/{total_tests})")
        return True
    else:
        print("\n❌ 所有测试失败，请检查硬件连接和配置")
        return False

if __name__ == '__main__':
    try:
        success = test_modbus_communication()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n测试被中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 未处理的异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
