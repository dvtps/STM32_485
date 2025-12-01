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
    # Add user sources here
    Core/App/main.c
    Core/App/motor_zdt.c
    Core/App/motor_state.c
    Core/App/log_system.c
    Core/App/modbus_rtu.c
    Core/App/modbus_gateway.c
    Drivers/BSP/EMM_V5/emm_v5.c
    Drivers/BSP/EMM_V5/emm_fifo.c
    Drivers/BSP/EMM_V5/emm_uart.c
    Drivers/BSP/LED/led.c
    Drivers/BSP/KEY/key.c
    Drivers/BSP/IWDG/iwdg.c
    Drivers/BSP/IWDG/iwdg_timer.c
    Drivers/SYSTEM/sys/sys.c
    Drivers/SYSTEM/delay/delay.c
    Drivers/SYSTEM/usart/usart.c
    Drivers/SYSTEM/usart/usart_dma.c
    Drivers/Middlewares/USMART/usmart.c
    Drivers/Middlewares/USMART/usmart_str.c
    Drivers/Middlewares/USMART/usmart_config.c
    Drivers/Middlewares/USMART/usmart_port.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/App
    Drivers/BSP/EMM_V5
    Drivers/BSP/LED
    Drivers/BSP/KEY
    Drivers/BSP/IWDG
    Drivers/SYSTEM/sys
    Drivers/SYSTEM/delay
    Drivers/SYSTEM/usart
    Drivers/Middlewares/USMART
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
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
