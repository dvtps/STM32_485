@echo off
chcp 65001 >nul
echo.
echo ============================================================
echo      STM32 Modbus硬件测试 - 一键启动
echo      STM32_485 Project
echo ============================================================
echo.

REM 检查Python是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ 错误: 未检测到Python，请先安装Python 3.x
    pause
    exit /b 1
)

echo ✅ Python已安装
echo.

REM 检查pymodbus是否安装
python -c "import pymodbus" >nul 2>&1
if errorlevel 1 (
    echo ⚠️  pymodbus未安装，正在安装...
    pip install pymodbus
    if errorlevel 1 (
        echo ❌ 安装失败，请手动运行: pip install pymodbus
        pause
        exit /b 1
    )
) else (
    echo ✅ pymodbus已安装
)

echo.
echo ============================================================
echo 请选择测试模式:
echo ============================================================
echo   1. 快速测试 (test_quick.py) - 推荐首次使用
echo   2. 交互式向导 (test_hardware_wizard.py) - 自动检测串口
echo   3. 简化测试 (test_modbus_simple.py)
echo   4. 完整测试 (test_modbus_gateway.py) - 7个测试用例
echo   5. 查看可用COM口
echo   0. 退出
echo ============================================================
echo.

set /p choice="请输入选项 (0-5): "

if "%choice%"=="1" (
    echo.
    echo [运行] 快速测试...
    python test_quick.py
    goto end
)

if "%choice%"=="2" (
    echo.
    echo [运行] 交互式向导...
    python test_hardware_wizard.py
    goto end
)

if "%choice%"=="3" (
    echo.
    echo [运行] 简化测试...
    python test_modbus_simple.py
    goto end
)

if "%choice%"=="4" (
    echo.
    echo [运行] 完整测试...
    python test_modbus_gateway.py
    goto end
)

if "%choice%"=="5" (
    echo.
    echo [可用COM口]
    python -c "import serial.tools.list_ports; ports = serial.tools.list_ports.comports(); print('\n'.join([f'{p.device} - {p.description}' for p in ports]))"
    echo.
    pause
    goto end
)

if "%choice%"=="0" (
    echo.
    echo 退出测试
    exit /b 0
)

echo.
echo ❌ 无效选项
pause

:end
echo.
echo ============================================================
echo 测试完成
echo ============================================================
pause
