cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(myproject_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(myproject
	myproject.c
	)

target_link_libraries(myproject pico_stdlib)

pico_enable_stdio_usb(myproject 1)
pico_enable_stdio_uart(myproject 1)

pico_add_extra_outputs(myproject)