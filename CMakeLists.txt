cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(vehicle_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# Shared code and hardware drivers
add_subdirectory(shared)
add_subdirectory(drivers)


# Modules
add_subdirectory(modules/motion)
add_subdirectory(modules/navigation)
add_subdirectory(modules/obstacle)
add_subdirectory(modules/telemetry)
add_subdirectory(modules/terrain)

add_executable(vehicle_controller app/main.c app/vehicle_controller/vehicle_controller.c)

target_include_directories(vehicle_controller PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/app
    ${CMAKE_CURRENT_LIST_DIR}/app/vehicle_controller
)

target_link_libraries(vehicle_controller PRIVATE
    pico_stdlib

    shared
    drivers

    telemetry
    motion
    navigation
    terrain
    obstacle
)

# USB serial output
pico_enable_stdio_usb(vehicle_controller 1)

# Disable UART serial output
pico_enable_stdio_uart(vehicle_controller 0)


# Generate .uf2, .bin, .hex, etc.
pico_add_extra_outputs(vehicle_controller)
