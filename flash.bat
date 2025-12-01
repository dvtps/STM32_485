@echo off
chcp 65001 >nul
echo.
echo ╔═══════════════════════════════════════════════════════════╗
echo ║         STM32固件烧录工具 (pyocd)                         ║
echo ║         STM32_485 Project V3.0                            ║
echo ╚═══════════════════════════════════════════════════════════╝
echo.

REM 设置固件路径
set FIRMWARE=build\Debug\STM32_485.elf
set TARGET=stm32f103c8

REM 检查固件是否存在
if not exist "%FIRMWARE%" (
    echo ❌ 错误: 固件文件不存在
    echo    路径: %FIRMWARE%
    echo.
    echo 请先编译固件:
    echo    cmake --build --preset Debug
    echo.
    pause
    exit /b 1
)

REM 显示固件信息
echo [固件信息]
for %%A in (%FIRMWARE%) do (
    echo   文件: %%~nxA
    echo   大小: %%~zA 字节
    echo   时间: %%~tA
)
echo.

REM 检查pyocd
echo [检查pyocd]
python -m pyocd --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ❌ pyocd未安装
    echo.
    echo 请安装pyocd:
    echo    pip install pyocd
    echo.
    pause
    exit /b 1
)
python -m pyocd --version
echo.

REM 列出已连接的调试器
echo [扫描调试器]
python -m pyocd list
echo.

REM 确认烧录
set /p confirm="确认烧录固件到 %TARGET%？(y/n): "
if /i not "%confirm%"=="y" (
    echo.
    echo 取消烧录
    pause
    exit /b 0
)

echo.
echo ============================================================
echo 开始烧录...
echo ============================================================
echo.

REM 执行烧录 (pyocd load命令，flash是load的别名)
python -m pyocd load -t %TARGET% %FIRMWARE%

if %errorlevel%==0 (
    echo.
    echo ============================================================
    echo ✅ 烧录成功！
    echo ============================================================
    echo.
    echo 下一步:
    echo   1. 按开发板复位键
    echo   2. 通过USART1查看调试输出 (115200bps)
    echo   3. 连接RS485模块准备测试
    echo.
) else (
    echo.
    echo ============================================================
    echo ❌ 烧录失败！
    echo ============================================================
    echo.
    echo 故障排查:
    echo   1. 检查ST-Link连接 (运行: python -m pyocd list)
    echo   2. 确认开发板供电正常
    echo   3. 尝试擦除芯片后重试
    echo.
)

pause
