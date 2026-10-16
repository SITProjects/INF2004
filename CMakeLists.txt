cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(vehicle_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# Shared code and hardware drivers
add_subdirectory(shared)
add_subdirectory(drivers)


# Modules
add_subdirectory(modules/line_follow)
add_subdirectory(modules/communication)
add_subdirectory(modules/motion)
add_subdirectory(modules/navigation)
add_subdirectory(modules/obstacle)
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

    communication
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

# Optional diagnostic firmware; does not change the vehicle application.
option(BUILD_IR_SERIAL_TEST "Enable the standalone IR serial test target" OFF)
if(BUILD_IR_SERIAL_TEST)
    add_executable(line_sensor_calibration EXCLUDE_FROM_ALL
        app/line_sensor_calibration/line_sensor_calibration.c
    )
    target_link_libraries(line_sensor_calibration PRIVATE
        pico_stdlib
        line_follow
    )
    pico_enable_stdio_usb(line_sensor_calibration 1)
    pico_enable_stdio_uart(line_sensor_calibration 0)
    pico_add_extra_outputs(line_sensor_calibration)
endif()
