cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32_485)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Enable CMake support for ASM and C languages
enable_language(C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ============ App层（纯业务逻辑编排） ============
    Core/App/main.c
    Core/App/motor_zdt.c
    Core/App/app_init.c
    Core/App/diagnostic.c
    Core/App/modbus_adapter.c
    Core/App/modbus_task.c
    Core/App/key_manager.c  # V3.2: 按键去抖状态机（App层）
    Core/App/error_handler.c  # V3.5: 错误处理系统
    Core/App/comm_monitor.c  # V3.5: 通信监控与超时管理
    
    # ============ Drivers/SYSTEM（系统级基础设施） ============
    Drivers/SYSTEM/sys/sys.c
    Drivers/SYSTEM/delay/delay.c
    Drivers/SYSTEM/fifo/fifo.c
    Drivers/SYSTEM/sys_timer/sys_timer.c  # V3.5: 非阻塞软件定时器
    Drivers/SYSTEM/usart/usart.c
    Drivers/SYSTEM/usart/usart_dma.c
    
    # ============ Drivers/BSP（板级硬件驱动） ============
    Drivers/BSP/EMM_V5/emm_v5.c
    Drivers/BSP/EMM_V5/emm_uart.c
    Drivers/BSP/EMM_V5/emm_state.c
    Drivers/BSP/LED/led.c
    Drivers/BSP/KEY/key.c
    Drivers/BSP/IWDG/iwdg.c
    Drivers/BSP/IWDG/iwdg_timer.c
    Drivers/BSP/MEM_POOL/mem_pool.c  # V3.5: 对象池内存管理
    Drivers/BSP/PROTOCOL_ROUTER/protocol_router.c  # V3.0: 协议路由器
    
    # ============ Drivers/Middlewares（中间件层） ============
    Drivers/Middlewares/LOGGER/logger.c
    Drivers/Middlewares/MODBUS/modbus_hal.c
    Drivers/Middlewares/MODBUS/modbus_rtu.c
    Drivers/Middlewares/MODBUS/modbus_gateway.c
    Drivers/Middlewares/MULTI_MOTOR/multi_motor_manager.c  # V3.1: 多电机管理中间件
    
    # USMART调试组件（Release构建时可移除以减少Flash占用）
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_str.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_config.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_port.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_interface.c>
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # App层（业务编排）
    Core/App
    
    # SYSTEM层（系统核心）
    Drivers/SYSTEM/sys
    Drivers/SYSTEM/delay
    Drivers/SYSTEM/fifo
    Drivers/SYSTEM/sys_timer  # V3.5: 非阻塞定时器
    Drivers/SYSTEM/usart
    
    # BSP层（硬件抽象）
    Drivers/BSP/EMM_V5
    Drivers/BSP/LED
    Drivers/BSP/KEY
    Drivers/BSP/IWDG
    Drivers/BSP/MEM_POOL  # V3.5: 对象池内存管理
    Drivers/BSP/PROTOCOL_ROUTER  # V3.0: 协议路由器
    
    # Middlewares层（可重用服务）
    Drivers/Middlewares/LOGGER
    Drivers/Middlewares/MODBUS
    Drivers/Middlewares/MULTI_MOTOR  # V3.1: 多电机管理中间件
    Drivers/Middlewares/USMART
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    $<$<CONFIG:Release>:NDEBUG>  # Release构建定义NDEBUG，禁用调试功能
    
    # V3.4功能开关（从app_config.h读取默认值，此处可覆盖）
    # FEATURE_USMART_ENABLE=1/0  # USMART调试组件（默认Release=0, Debug=1）
    # FEATURE_MODBUS_ENABLE=1/0  # Modbus RTU协议栈（默认=1）
    # FEATURE_MULTI_MOTOR_BATCH_ENABLE=1/0  # 多电机批处理函数（默认=1）
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Suppress RWX segment warning (normal for embedded systems)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,--no-warn-rwx-segments
)
