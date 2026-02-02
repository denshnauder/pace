cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RC_C_Com)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE)

# --- 重点修改：去掉了 Core/ 前缀 ---
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 1. 系统源文件 (直接在 Src 目录下)
    ${CMAKE_SOURCE_DIR}/Src/main.c
    ${CMAKE_SOURCE_DIR}/Src/gpio.c
    ${CMAKE_SOURCE_DIR}/Src/dma.c
    ${CMAKE_SOURCE_DIR}/Src/usart.c
    ${CMAKE_SOURCE_DIR}/Src/stm32f4xx_it.c
    ${CMAKE_SOURCE_DIR}/Src/stm32f4xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Src/syscalls.c
    # 启动文件 (通常在根目录或 Startup 文件夹，如果报错找不到，请确认它的位置)
    ${CMAKE_SOURCE_DIR}/startup_stm32f407xx.s
    
    # 2. 你的 UserCode
    ${CMAKE_SOURCE_DIR}/UserCode/ca_com.c
    ${CMAKE_SOURCE_DIR}/UserCode/drv_uart.c
    ${CMAKE_SOURCE_DIR}/UserCode/remote_control.c
    ${CMAKE_SOURCE_DIR}/UserCode/usefulFunction.c
    
    # 3. BSP
    ${CMAKE_SOURCE_DIR}/bsp/boards/bsp_rc.c
)

# --- 头文件路径也去掉了 Core/ ---
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Inc   # 改成了 Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    # 用户头文件
    ${CMAKE_SOURCE_DIR}/bsp/boards
    ${CMAKE_SOURCE_DIR}/UserCode
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 宏定义留空，由 CubeMX 模块自动处理
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)