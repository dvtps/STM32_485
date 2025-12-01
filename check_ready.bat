@echo off
chcp 65001 >nul
echo.
echo ╔═══════════════════════════════════════════════════════════╗
echo ║     STM32硬件测试准备检查器                               ║
echo ║     STM32_485 Project V3.0                                ║
echo ╚═══════════════════════════════════════════════════════════╝
echo.

REM 检查1: 固件是否已编译
echo [检查 1/5] 固件编译状态
if exist "build\Debug\STM32_485.elf" (
    echo   ✅ 固件已编译: build\Debug\STM32_485.elf
    for %%A in (build\Debug\STM32_485.elf) do (
        echo   文件大小: %%~zA 字节
        echo   修改时间: %%~tA
    )
) else (
    echo   ❌ 固件未编译！请先运行: cmake --build --preset Debug
    pause
    exit /b 1
)
echo.

REM 检查2: ST-Link驱动
echo [检查 2/5] ST-Link驱动状态
where ST-LINK_CLI >nul 2>&1
if %errorlevel%==0 (
    echo   ✅ ST-Link命令行工具已安装
) else (
    echo   ⚠️  ST-Link命令行工具未找到
    echo   建议安装: STM32 ST-LINK Utility 或 STM32CubeProgrammer
)
echo.

REM 检查3: Python环境
echo [检查 3/5] Python测试环境
python --version >nul 2>&1
if %errorlevel%==0 (
    echo   ✅ Python已安装
    python --version 2>&1 | findstr /C:"Python"
) else (
    echo   ❌ Python未安装！请安装Python 3.x
)

python -c "import pymodbus" >nul 2>&1
if %errorlevel%==0 (
    echo   ✅ pymodbus库已安装
) else (
    echo   ⚠️  pymodbus未安装，请运行: pip install pymodbus
)
echo.

REM 检查4: 测试脚本
echo [检查 4/5] 测试脚本状态
set script_count=0
if exist "Docs\test_quick.py" (
    echo   ✅ test_quick.py
    set /a script_count+=1
)
if exist "Docs\test_modbus_simple.py" (
    echo   ✅ test_modbus_simple.py
    set /a script_count+=1
)
if exist "Docs\test_modbus_gateway.py" (
    echo   ✅ test_modbus_gateway.py
    set /a script_count+=1
)
if exist "Docs\test_hardware_wizard.py" (
    echo   ✅ test_hardware_wizard.py
    set /a script_count+=1
)
echo   共找到 %script_count% 个测试脚本
echo.

REM 检查5: 可用COM口
echo [检查 5/5] 可用COM口
echo   正在扫描...
python -c "import serial.tools.list_ports; ports = serial.tools.list_ports.comports(); [print(f'   ✅ {p.device} - {p.description}') for p in ports] if ports else print('   ❌ 未找到COM口')" 2>nul
if %errorlevel% neq 0 (
    echo   ⚠️  无法扫描COM口（需要pyserial库）
)
echo.

REM 总结
echo ╔═══════════════════════════════════════════════════════════╗
echo ║                     检查完成                              ║
echo ╚═══════════════════════════════════════════════════════════╝
echo.
echo 下一步操作:
echo   1. 烧录固件: 参考 Docs\FLASH_GUIDE.md
echo   2. 连接硬件: STM32 + RS485模块
echo   3. 运行测试: cd Docs; python test_quick.py
echo.
pause
