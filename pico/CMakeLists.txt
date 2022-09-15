cmake_minimum_required(VERSION 3.12)

include(sdk/pico-sdk/external/pico_sdk_import.cmake)

project(spot_micro_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


pico_sdk_init()

link_directories(sdk/micro_ros_raspberrypi_pico_sdk/libmicroros)

add_library(pico_uart_transport 
 sdk/micro_ros_raspberrypi_pico_sdk/pico_uart_transports.h
 sdk/micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c 
)

add_executable(pico_9685_subscriber src/pico_9685_subscriber.cpp sdk/micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c)
target_link_libraries(pico_9685_subscriber pico_stdlib microros pico_uart_transport)
target_include_directories(pico_9685_subscriber PUBLIC sdk/micro_ros_raspberrypi_pico_sdk/libmicroros/include)


SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_9685_subscriber 1)
pico_enable_stdio_uart(pico_9685_subscriber 0)


add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_9685_subscriber)


