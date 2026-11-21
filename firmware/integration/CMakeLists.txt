# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Copia aquí pico_sdk_import.cmake desde $PICO_SDK_PATH/external/
include(pico_sdk_import.cmake)

project(autopilot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(autopilot main.cpp)

target_link_libraries(autopilot
    pico_stdlib
    pico_multicore
    hardware_uart
    hardware_i2c
    hardware_pwm
    hardware_watchdog
    hardware_sync
    hardware_dma
    hardware_clocks)

# USB para la telemetría; el UART0 queda libre para el i-BUS (GP1)
pico_enable_stdio_usb(autopilot 1)
pico_enable_stdio_uart(autopilot 0)

pico_add_extra_outputs(autopilot)
