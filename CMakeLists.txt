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
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-pedantic -Wno-int-conversion)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
                Drivers/BSP/Y_V2/emm_v5_parser.c       # ============ App层（纯业务逻辑编排） ============
    Core/App/main.c
    Core/App/app_init.c
    Core/App/app_tasks.c
    Core/App/error_handler.c
    Core/App/printer_axis.c
    Core/App/motor_sequence.c    # 电机运动序列控制模块
    Core/App/realtime_motor.c   # 实时电机控制核心
    Core/App/motor_monitor.c   # 电机监控与反馈系统
    
    # ============ Drivers/SYSTEM（系统级基础设施） ============
    Drivers/SYSTEM/sys/sys.c
    Drivers/SYSTEM/delay/delay.c
    Drivers/SYSTEM/fifo/fifo.c
    Drivers/SYSTEM/sys_timer/sys_timer.c  # V3.5: 非阻塞软件定时器
    Drivers/SYSTEM/usart/usart.c          # V3.6: 集成DMA收发（降低CPU占用95%）
    Drivers/SYSTEM/tim2_rt/tim2_rt.c      # V3.6: TIM2实时定时器（10kHz控制周期）
    
    # ============ Drivers/BSP（板级硬件驱动） ============
    Drivers/BSP/Y_V2/y_v2.c            # V3.0: Y系列X固件协议实现
    Drivers/BSP/Y_V2/emm_uart.c        # V3.0: 统一RS485通信层
    # Drivers/BSP/Y_V2/emm_state.c       # 如未用可注释
    Drivers/BSP/Y_V2/emm_v5_parser.c   # 响应帧解析器
    Drivers/BSP/LED/led.c         # 保留LED驱动
        Drivers/BSP/KEY/key.c       # 按键驱动，修复 key_init 链接错误
    Drivers/BSP/IWDG/iwdg.c       # 保留看门狗
    # Drivers/BSP/IWDG/iwdg_timer.c # 如未用可注释
    
    # ============ Drivers/Middlewares（中间件层） ============
    Drivers/Middlewares/LOGGER/logger.c  # 保留日志系统
    
    # USMART调试组件
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_str.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_config.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_port.c>
    $<$<CONFIG:Debug>:Drivers/Middlewares/USMART/usmart_interface.c>
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/App
    Core/Inc
    Drivers/SYSTEM/delay
    Drivers/SYSTEM/fifo
    Drivers/SYSTEM/sys_timer  # V3.5: 非阻塞定时器
    Drivers/SYSTEM/usart
    Drivers/SYSTEM/tim2_rt    # V3.6: TIM2实时定时器
    
    # BSP层（硬件抽象）
    Drivers/BSP/Y_V2          # V3.0: Y系列X固件协议驱动
    Drivers/BSP/LED
    Drivers/BSP/KEY
    Drivers/BSP/IWDG
    
    # Middlewares层（可重用服务）
    Drivers/Middlewares/LOGGER
    Drivers/Middlewares/USMART

    # SYSTEM层补充sys.h路径
    Drivers/SYSTEM/sys

    # 标准库头文件路径（gcc-arm-none-eabi）
    $ENV{GNU_ARM_TOOLCHAIN}/lib/gcc/arm-none-eabi/13.3.1/include
    $ENV{GNU_ARM_TOOLCHAIN}/lib/gcc/arm-none-eabi/13.3.1/include-fixed
    $ENV{GNU_ARM_TOOLCHAIN}/arm-none-eabi/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        $<$<CONFIG:Debug>:FEATURE_USMART_ENABLE=1>
        $<$<CONFIG:Release>:FEATURE_USMART_ENABLE=0>
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
