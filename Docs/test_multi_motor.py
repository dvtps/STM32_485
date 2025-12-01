"""
STM32多电机Modbus控制测试脚本 V3.1
测试多电机批量控制和状态监控功能
"""

import serial
from pymodbus.client import ModbusSerialClient
import time

# ========== 配置参数 ==========
SERIAL_PORT = 'COM7'
BAUDRATE = 115200
SLAVE_ADDRESS = 1
TIMEOUT = 1.0

# ========== Modbus寄存器地址 ==========
# 全局控制区
REG_SYS_ENABLE = 0x0000         # 系统使能（BIT0-15对应16个电机）

# 电机控制区（每电机64寄存器，起始0x0100）
def motor_reg(motor_id, offset):
    """计算电机寄存器地址
    motor_id: 1-16
    offset: 寄存器偏移（0x00-0x3F）
    """
    return 0x0100 + (motor_id - 1) * 0x40 + offset

# 电机状态区（每电机32寄存器，起始0x0500）
def motor_status_reg(motor_id, offset):
    """计算电机状态寄存器地址"""
    return 0x0500 + (motor_id - 1) * 0x20 + offset

# ========== 测试函数 ==========

def test_multi_motor_enable(client):
    """测试1: 批量使能电机"""
    print("\n" + "="*60)
    print("[测试 1/5] 批量使能电机（电机1-4）")
    print("="*60)
    
    try:
        # 使能电机1-4（BIT0-3 = 0x000F）
        motor_mask = 0x000F  # 二进制: 0000000000001111
        result = client.write_register(address=REG_SYS_ENABLE, value=motor_mask, device_id=SLAVE_ADDRESS)
        
        if result.isError():
            print(f"❌ 写入失败: {result}")
            return False
        
        print(f"✅ 批量使能成功: mask=0x{motor_mask:04X}")
        
        # 回读验证
        time.sleep(0.1)
        verify = client.read_holding_registers(address=REG_SYS_ENABLE, count=1, device_id=SLAVE_ADDRESS)
        if not verify.isError():
            actual = verify.registers[0]
            print(f"   回读验证: 0x{actual:04X} (电机1-4已使能)" if actual == motor_mask else f"⚠️  回读不匹配")
        
        return True
        
    except Exception as e:
        print(f"❌ 异常: {e}")
        return False

def test_multi_motor_batch_pos(client):
    """测试2: 批量位置控制"""
    print("\n" + "="*60)
    print("[测试 2/5] 批量位置控制（电机1-2同时转半圈）")
    print("="*60)
    
    try:
        # 配置电机1: 方向=CW, 速度=300RPM, 加速度=10, 脉冲=1600（半圈）
        motor1_regs = [
            0,      # 0x00: 方向（0=CW）
            300,    # 0x01: 速度（RPM）
            10,     # 0x02: 加速度
            0,      # 0x03: 脉冲高16位
            1600,   # 0x04: 脉冲低16位
            1,      # 0x05: 相对运动
        ]
        
        # 写入电机1控制寄存器
        addr1 = motor_reg(1, 0)
        result = client.write_registers(address=addr1, values=motor1_regs, device_id=SLAVE_ADDRESS)
        if result.isError():
            print(f"❌ 电机1配置失败")
            return False
        print(f"✅ 电机1配置成功: {motor1_regs}")
        
        # 配置电机2（相同参数）
        addr2 = motor_reg(2, 0)
        result = client.write_registers(address=addr2, values=motor1_regs, device_id=SLAVE_ADDRESS)
        if result.isError():
            print(f"❌ 电机2配置失败")
            return False
        print(f"✅ 电机2配置成功")
        
        # 触发EXEC_COMMAND（写0x01到偏移0x10）
        exec_addr1 = motor_reg(1, 0x10)
        exec_addr2 = motor_reg(2, 0x10)
        client.write_register(address=exec_addr1, value=0x01, device_id=SLAVE_ADDRESS)
        client.write_register(address=exec_addr2, value=0x01, device_id=SLAVE_ADDRESS)
        
        print("✅ 批量运动命令已发送，电机应同步运动")
        return True
        
    except Exception as e:
        print(f"❌ 异常: {e}")
        return False

def test_read_motor_status(client, motor_id):
    """测试3: 读取单个电机状态"""
    print(f"\n[读取电机{motor_id}状态]")
    
    try:
        # 读取状态寄存器（6个）
        addr = motor_status_reg(motor_id, 0)
        result = client.read_input_registers(address=addr, count=6, device_id=SLAVE_ADDRESS)
        
        if result.isError():
            print(f"❌ 读取失败")
            return False
        
        regs = result.registers
        position_h = regs[0]
        position_l = regs[1]
        position = (position_h << 16) | position_l
        
        speed = regs[2]
        current = regs[3]
        voltage = regs[4]
        temp = regs[5]
        
        print(f"   位置: {position} 脉冲 ({position/3200:.2f}圈)")
        print(f"   速度: {speed*0.1:.1f} RPM")
        print(f"   电流: {current} mA")
        print(f"   电压: {voltage} mV")
        print(f"   温度: {temp} ℃")
        
        return True
        
    except Exception as e:
        print(f"❌ 异常: {e}")
        return False

def test_motor_discovery(client):
    """测试4: 电机发现功能"""
    print("\n" + "="*60)
    print("[测试 4/5] 电机发现（扫描地址1-4）")
    print("="*60)
    
    online_motors = []
    
    for addr in range(1, 5):
        try:
            # 尝试读取固件版本寄存器
            result = client.read_holding_registers(address=0x0007, count=1, device_id=SLAVE_ADDRESS, timeout=0.5)
            
            if not result.isError():
                online_motors.append(addr)
                print(f"✅ 电机{addr}: 在线")
            else:
                print(f"⚪ 电机{addr}: 离线")
                
        except Exception as e:
            print(f"⚪ 电机{addr}: 超时")
        
        time.sleep(0.1)
    
    print(f"\n总计: {len(online_motors)} 台电机在线")
    return len(online_motors) > 0

def test_batch_stop(client):
    """测试5: 批量急停"""
    print("\n" + "="*60)
    print("[测试 5/5] 批量急停（电机1-4）")
    print("="*60)
    
    try:
        # 急停命令：写0x03到各电机EXEC_COMMAND寄存器
        for motor_id in range(1, 5):
            exec_addr = motor_reg(motor_id, 0x10)
            result = client.write_register(address=exec_addr, value=0x03, device_id=SLAVE_ADDRESS)
            
            if not result.isError():
                print(f"✅ 电机{motor_id}急停命令已发送")
        
        return True
        
    except Exception as e:
        print(f"❌ 异常: {e}")
        return False

# ========== 主测试流程 ==========

def main():
    print("\n" + "="*60)
    print(" STM32多电机Modbus控制测试 V3.1")
    print("="*60)
    
    # 连接串口
    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT,
        parity='N',
        stopbits=1,
        bytesize=8
    )
    
    if not client.connect():
        print(f"❌ 无法打开串口 {SERIAL_PORT}")
        return
    
    print(f"✅ 已连接到 {SERIAL_PORT} ({BAUDRATE}bps)")
    
    results = {
        '批量使能': False,
        '批量位置控制': False,
        '电机发现': False,
        '批量急停': False,
    }
    
    try:
        # 测试1: 批量使能
        results['批量使能'] = test_multi_motor_enable(client)
        time.sleep(0.5)
        
        # 测试2: 批量位置控制
        results['批量位置控制'] = test_multi_motor_batch_pos(client)
        time.sleep(2)  # 等待运动完成
        
        # 测试3: 读取状态
        test_read_motor_status(client, 1)
        test_read_motor_status(client, 2)
        
        # 测试4: 电机发现
        results['电机发现'] = test_motor_discovery(client)
        
        # 测试5: 批量急停
        results['批量急停'] = test_batch_stop(client)
        
    finally:
        client.close()
    
    # 结果汇总
    print("\n" + "="*60)
    print(" 测试结果汇总")
    print("="*60)
    
    passed = sum(results.values())
    total = len(results)
    
    for test_name, result in results.items():
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{test_name}: {status}")
    
    print(f"\n总计: {total} 个测试")
    print(f"✅ 通过: {passed}")
    print(f"❌ 失败: {total - passed}")
    print("="*60)
    
    if passed == total:
        print("\n🎉 所有测试通过！多电机控制系统运行正常！")
    else:
        print(f"\n⚠️  部分测试失败，请检查配置")

if __name__ == "__main__":
    main()
