cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Please set the PICO_SDK_PATH environment variable to point to the Raspberry Pi Pico SDK")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(example C CXX ASM)

pico_sdk_init()

add_executable(example
        example.c
        )

# pull in common dependencies
target_link_libraries(example pico_stdlib)

# enable usb output, disable uart output
pico_enable_stdio_usb(example 1)
pico_enable_stdio_uart(example 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(example)
