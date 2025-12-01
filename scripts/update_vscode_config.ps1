# 自动更新 VS Code IntelliSense 配置脚本
# 功能: 检测 arm-none-eabi-gcc 路径并更新 .vscode/c_cpp_properties.json

$ErrorActionPreference = "Stop"

Write-Host "=== VS Code IntelliSense Config Update Tool ===" -ForegroundColor Cyan

# 检测 arm-none-eabi-gcc 路径
Write-Host "`n[1/5] 检测工具链路径..." -ForegroundColor Yellow
$gccPath = (Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue).Source

if (-not $gccPath) {
    Write-Host "❌ 错误: 未找到 arm-none-eabi-gcc" -ForegroundColor Red
    Write-Host "请确保工具链已安装并添加到 PATH 环境变量" -ForegroundColor Red
    Write-Host "参考路径: C:\ST\STM32CubeCLT_1.19.0\GNU-tools-for-STM32\bin" -ForegroundColor Gray
    exit 1
}

Write-Host "✅ 找到工具链: $gccPath" -ForegroundColor Green

# 提取工具链根目录
$toolchainBin = Split-Path $gccPath
$toolchainRoot = Split-Path $toolchainBin
Write-Host "   根目录: $toolchainRoot" -ForegroundColor Gray

# 验证标准库路径存在
Write-Host "`n[2/5] 验证标准库路径..." -ForegroundColor Yellow
$stdInclude = Join-Path $toolchainRoot "arm-none-eabi\include\stdio.h"
if (-not (Test-Path $stdInclude)) {
    Write-Host "❌ 错误: 未找到 stdio.h" -ForegroundColor Red
    Write-Host "预期路径: $stdInclude" -ForegroundColor Gray
    exit 1
}
Write-Host "✅ 标准库路径有效" -ForegroundColor Green

# 查找 GCC 版本
Write-Host "`n[3/5] 检测 GCC 版本..." -ForegroundColor Yellow
$gccVersion = & $gccPath --version | Select-Object -First 1
Write-Host "   $gccVersion" -ForegroundColor Gray

# 解析版本号（如 13.3.1）
$versionMatch = $gccVersion -match '(\d+\.\d+\.\d+)'
if ($versionMatch) {
    $version = $Matches[1]
    Write-Host "✅ 版本: $version" -ForegroundColor Green
} else {
    Write-Host "⚠️  警告: 无法解析版本号，使用默认 13.3.1" -ForegroundColor Yellow
    $version = "13.3.1"
}

# 构建配置对象
Write-Host "`n[4/5] 生成配置..." -ForegroundColor Yellow
$config = @{
    configurations = @(
        @{
            name = "STM32"
            compilerPath = $gccPath.Replace('\', '/')
            compileCommands = "`${workspaceFolder}/build/Debug/compile_commands.json"
            intelliSenseMode = "gcc-arm"
            cStandard = "c11"
            cppStandard = "c++17"
            defines = @(
                "USE_HAL_DRIVER",
                "STM32F103xB",
                "DEBUG"
            )
            includePath = @(
                "`${workspaceFolder}/**",
                "$toolchainRoot/arm-none-eabi/include".Replace('\', '/'),
                "$toolchainRoot/arm-none-eabi/include/c++/$version".Replace('\', '/'),
                "$toolchainRoot/arm-none-eabi/include/c++/$version/arm-none-eabi".Replace('\', '/'),
                "$toolchainRoot/lib/gcc/arm-none-eabi/$version/include".Replace('\', '/'),
                "$toolchainRoot/lib/gcc/arm-none-eabi/$version/include-fixed".Replace('\', '/')
            )
        }
    )
    version = 4
}

# 确保 .vscode 目录存在
$vscodeDir = ".vscode"
if (-not (Test-Path $vscodeDir)) {
    New-Item -ItemType Directory -Path $vscodeDir | Out-Null
    Write-Host "✅ 创建目录: $vscodeDir" -ForegroundColor Green
}

# 写入配置文件
Write-Host "`n[5/5] 写入配置文件..." -ForegroundColor Yellow
$configFile = Join-Path $vscodeDir "c_cpp_properties.json"
$config | ConvertTo-Json -Depth 10 | Set-Content $configFile -Encoding UTF8

Write-Host "✅ 配置已更新: $configFile" -ForegroundColor Green

# Display summary
Write-Host ""
Write-Host "=== Configuration Summary ===" -ForegroundColor Cyan
Write-Host "Compiler: $gccPath" -ForegroundColor Gray
Write-Host "Version:  $version" -ForegroundColor Gray
Write-Host ""
Write-Host "Done! Configuration updated successfully." -ForegroundColor Green
Write-Host "Next: Restart VS Code or reset IntelliSense" -ForegroundColor Yellow
