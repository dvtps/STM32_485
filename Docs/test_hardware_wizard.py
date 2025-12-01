#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Modbus硬件测试向导
自动检测串口、引导硬件连接、执行测试
"""

import sys
import time
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# 颜色定义（Windows终端）
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_header(text):
    print(f"\n{Colors.BOLD}{Colors.CYAN}{'='*60}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}{text.center(60)}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}{'='*60}{Colors.RESET}\n")

def print_success(text):
    print(f"{Colors.GREEN}✅ {text}{Colors.RESET}")

def print_error(text):
    print(f"{Colors.RED}❌ {text}{Colors.RESET}")

def print_warning(text):
    print(f"{Colors.YELLOW}⚠️  {text}{Colors.RESET}")

def print_info(text):
    print(f"{Colors.BLUE}ℹ️  {text}{Colors.RESET}")

def list_serial_ports():
    """列出所有可用串口"""
    ports = serial.tools.list_ports.comports()
    available = []
    
    print_info("扫描可用串口...")
    if not ports:
        print_error("未检测到任何串口设备")
        return []
    
    print(f"\n{Colors.BOLD}可用串口列表:{Colors.RESET}")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {Colors.CYAN}{port.device}{Colors.RESET} - {port.description}")
        if port.manufacturer:
            print(f"     制造商: {port.manufacturer}")
        available.append(port.device)
    
    return available

def select_serial_port(available_ports):
    """选择串口"""
    if not available_ports:
        return None
    
    if len(available_ports) == 1:
        print_info(f"自动选择唯一串口: {available_ports[0]}")
        return available_ports[0]
    
    while True:
        try:
            choice = input(f"\n{Colors.YELLOW}请选择串口编号 (1-{len(available_ports)}) 或输入完整COM口名称: {Colors.RESET}")
            
            # 尝试作为编号解析
            if choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(available_ports):
                    return available_ports[idx]
            
            # 尝试作为COM口名称解析
            choice_upper = choice.upper()
            if choice_upper in available_ports:
                return choice_upper
            
            # 尝试添加COM前缀
            if not choice_upper.startswith('COM'):
                choice_upper = 'COM' + choice_upper
                if choice_upper in available_ports:
                    return choice_upper
            
            print_error("无效选择，请重试")
        
        except KeyboardInterrupt:
            print("\n\n操作已取消")
            return None

def check_hardware_connection():
    """硬件连接检查清单"""
    print_header("硬件连接检查清单")
    
    checklist = [
        ("STM32已通过USB线连接并上电", "LED应闪烁或常亮"),
        ("USART2通过RS485模块连接到PC", "PA2(TX)→RS485 TXD, PA3(RX)→RS485 RXD"),
        ("RS485 A/B线正确连接", "A-A, B-B（不要交叉）"),
        ("（可选）USART1连接USB-TTL查看调试日志", "PA9(TX)→USB-TTL RX, 115200bps"),
    ]
    
    for i, (item, detail) in enumerate(checklist, 1):
        print(f"{i}. {Colors.BOLD}{item}{Colors.RESET}")
        print(f"   {Colors.CYAN}{detail}{Colors.RESET}")
    
    print()
    response = input(f"{Colors.YELLOW}硬件连接已就绪？(y/n): {Colors.RESET}").lower()
    return response == 'y' or response == 'yes' or response == ''

def quick_connection_test(port, baudrate=115200, slave_addr=1):
    """快速连接测试"""
    print_header("快速连接测试")
    print_info(f"测试参数: {port}, {baudrate}bps, 从机地址={slave_addr}")
    
    try:
        client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            timeout=2.0,
            parity='N',
            stopbits=1,
            bytesize=8
        )
        
        if not client.connect():
            print_error(f"无法打开串口 {port}")
            print_warning("可能原因:")
            print("  - 串口被其他程序占用（串口助手、PuTTY等）")
            print("  - 权限不足（Linux需要sudo或加入dialout组）")
            print("  - USB线接触不良")
            return None
        
        print_success(f"串口 {port} 已打开")
        
        # 测试1: 读取固件版本（地址0x0007）
        print_info("尝试读取固件版本（地址0x0007）...")
        result = client.read_holding_registers(address=0x0007, count=1, slave=slave_addr)
        
        if result.isError():
            print_error(f"Modbus通信失败: {result}")
            print_warning("可能原因:")
            print("  - STM32固件未烧录或未启动")
            print("  - FEATURE_MODBUS_ENABLE=0（未启用Modbus）")
            print("  - RS485 A/B线接反")
            print("  - 从机地址不匹配（当前测试地址=1）")
            client.close()
            return None
        
        fw_version = result.registers[0]
        major = fw_version >> 8
        minor = fw_version & 0xFF
        print_success(f"Modbus通信成功！固件版本: V{major}.{minor}")
        
        return client
    
    except Exception as e:
        print_error(f"异常: {e}")
        return None

def run_basic_tests(client, slave_addr=1):
    """运行基础测试"""
    print_header("基础功能测试")
    
    tests_passed = 0
    tests_total = 0
    
    # 测试1: 读取系统信息
    tests_total += 1
    print(f"\n{Colors.BOLD}[测试 1/4] 读取系统信息{Colors.RESET}")
    try:
        result = client.read_holding_registers(address=0x0007, count=3, slave=slave_addr)
        if not result.isError():
            fw_ver = result.registers[0]
            hw_ver = result.registers[1]
            device_id = result.registers[2]
            print_success("读取成功")
            print(f"  固件版本: V{fw_ver>>8}.{fw_ver&0xFF}")
            print(f"  硬件版本: 0x{hw_ver:04X}")
            print(f"  设备ID: {device_id}")
            tests_passed += 1
        else:
            print_error(f"失败: {result}")
    except Exception as e:
        print_error(f"异常: {e}")
    
    # 测试2: 写单个寄存器
    tests_total += 1
    print(f"\n{Colors.BOLD}[测试 2/4] 写单个寄存器（设置电机速度）{Colors.RESET}")
    try:
        result = client.write_register(address=0x0103, value=5000, slave=slave_addr)
        if not result.isError():
            # 回读验证
            result = client.read_holding_registers(address=0x0103, count=1, slave=slave_addr)
            if not result.isError() and result.registers[0] == 5000:
                print_success("写入成功，回读验证通过")
                print(f"  地址0x0103 = {result.registers[0]} (500.0 RPM)")
                tests_passed += 1
            else:
                print_error("写入成功但回读验证失败")
        else:
            print_error(f"失败: {result}")
    except Exception as e:
        print_error(f"异常: {e}")
    
    # 测试3: 写多个寄存器
    tests_total += 1
    print(f"\n{Colors.BOLD}[测试 3/4] 写多个寄存器（批量配置）{Colors.RESET}")
    try:
        values = [1, 1, 0, 3000]  # ENABLE, CTRL_MODE, DIRECTION, SPEED
        result = client.write_registers(address=0x0100, values=values, slave=slave_addr)
        if not result.isError():
            # 回读验证
            result = client.read_holding_registers(address=0x0100, count=4, slave=slave_addr)
            if not result.isError():
                print_success("批量写入成功")
                print(f"  ENABLE={result.registers[0]}, MODE={result.registers[1]}, "
                      f"DIR={result.registers[2]}, SPEED={result.registers[3]}")
                tests_passed += 1
            else:
                print_error("写入成功但回读验证失败")
        else:
            print_error(f"失败: {result}")
    except Exception as e:
        print_error(f"异常: {e}")
    
    # 测试4: 读输入寄存器（状态区）
    tests_total += 1
    print(f"\n{Colors.BOLD}[测试 4/4] 读输入寄存器（电机状态）{Colors.RESET}")
    try:
        result = client.read_input_registers(address=0x0500, count=6, slave=slave_addr)
        if not result.isError():
            pos_h, pos_l, speed, current, voltage, temp = result.registers
            position = (pos_h << 16) | pos_l
            if position & 0x80000000:
                position -= 0x100000000
            print_success("读取成功")
            print(f"  实时位置: {position/10.0:.1f}°")
            print(f"  实时速度: {speed/10.0:.1f} RPM")
            print(f"  实时电流: {current} mA")
            print(f"  实时电压: {voltage} mV")
            print(f"  实时温度: {temp}℃")
            tests_passed += 1
        else:
            print_error(f"失败: {result}")
    except Exception as e:
        print_error(f"异常: {e}")
    
    # 测试结果汇总
    print_header("测试结果")
    print(f"总计: {tests_total} 个测试")
    print_success(f"通过: {tests_passed}")
    if tests_total - tests_passed > 0:
        print_error(f"失败: {tests_total - tests_passed}")
    
    return tests_passed, tests_total

def motor_control_test(client, slave_addr=1):
    """电机控制测试（需要实际电机）"""
    print_header("电机控制测试（需要连接实际电机）")
    
    response = input(f"{Colors.YELLOW}是否已连接电机并准备测试？(y/n): {Colors.RESET}").lower()
    if response not in ['y', 'yes']:
        print_warning("跳过电机控制测试")
        return
    
    print_info("准备执行电机1位置控制（1圈运动）...")
    
    try:
        # 步骤1: 使能电机
        print_info("步骤1: 使能电机...")
        result = client.write_register(address=0x0100, value=1, slave=slave_addr)
        if result.isError():
            print_error("使能失败")
            return
        print_success("电机已使能")
        time.sleep(0.2)
        
        # 步骤2: 配置运动参数
        print_info("步骤2: 配置运动参数（速度300 RPM, 位置3200脉冲=1圈）...")
        values = [
            1,      # ENABLE
            1,      # CTRL_MODE = 位置模式
            0,      # DIRECTION = 顺时针
            3000,   # SPEED = 300.0 RPM
            10,     # ACCELERATION
            10,     # DECELERATION
            0,      # POSITION_H
            3200    # POSITION_L (1圈)
        ]
        result = client.write_registers(address=0x0100, values=values, slave=slave_addr)
        if result.isError():
            print_error("配置失败")
            return
        print_success("运动参数已配置")
        
        # 步骤3: 执行运动命令
        print_info("步骤3: 执行位置运动命令...")
        CMD_POS_MOVE = 0x0003
        result = client.write_register(address=0x0108, value=CMD_POS_MOVE, slave=slave_addr)
        if result.isError():
            print_error("执行失败")
            return
        
        print_success("位置运动命令已发送")
        print_warning("电机应该正在运动... 请观察实际电机")
        
        # 等待运动完成
        print_info("等待运动完成（预计5秒）...")
        for i in range(5):
            time.sleep(1)
            print(f"  {i+1}/5 秒...")
        
        print_success("电机控制测试完成")
        
    except Exception as e:
        print_error(f"异常: {e}")

def main():
    """主程序"""
    print(f"\n{Colors.BOLD}{Colors.GREEN}")
    print("╔═══════════════════════════════════════════════════════════╗")
    print("║         STM32 Modbus硬件测试向导 V1.0                    ║")
    print("║         STM32_485 Project - 2025-12-01                   ║")
    print("╚═══════════════════════════════════════════════════════════╝")
    print(f"{Colors.RESET}")
    
    # 步骤1: 扫描串口
    available_ports = list_serial_ports()
    if not available_ports:
        print_error("未检测到串口设备，请检查硬件连接")
        return 1
    
    # 步骤2: 选择串口
    selected_port = select_serial_port(available_ports)
    if not selected_port:
        return 1
    
    print_success(f"已选择串口: {selected_port}")
    
    # 步骤3: 硬件连接检查
    if not check_hardware_connection():
        print_warning("请完成硬件连接后重新运行测试")
        return 1
    
    # 步骤4: 快速连接测试
    client = quick_connection_test(selected_port)
    if not client:
        print_error("连接测试失败，请检查硬件和配置")
        print_info("\n故障排查建议:")
        print("  1. 确认STM32已烧录最新固件（build/Debug/STM32_485.elf）")
        print("  2. 打开串口助手监听USART1(115200bps)查看调试日志")
        print("  3. 检查app_config.h中FEATURE_MODBUS_ENABLE=1")
        print("  4. 验证RS485接线：A-A, B-B")
        return 1
    
    try:
        # 步骤5: 运行基础测试
        passed, total = run_basic_tests(client)
        
        if passed < total:
            print_warning(f"\n部分测试失败 ({passed}/{total})，建议检查配置")
        else:
            print_success(f"\n所有基础测试通过！({passed}/{total})")
        
        # 步骤6: 电机控制测试（可选）
        if passed == total:
            motor_control_test(client)
        
    finally:
        client.close()
        print_info("\n串口已关闭")
    
    print_header("测试完成")
    print_info("如需完整测试，请运行: python test_modbus_gateway.py")
    
    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}测试被中断{Colors.RESET}")
        sys.exit(1)
    except Exception as e:
        print_error(f"未处理的异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
