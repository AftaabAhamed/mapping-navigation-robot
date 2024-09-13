cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(first_attempt C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()

link_directories(libmicroros)
add_executable(first_attempt
    first_attempt.cpp
    pico_uart_transport.c

)
target_link_libraries(first_attempt
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_gpio
    hardware_pio
    pico_time
    microros
)

target_include_directories(first_attempt PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(first_attempt 1)
pico_enable_stdio_uart(first_attempt 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(first_attempt)
