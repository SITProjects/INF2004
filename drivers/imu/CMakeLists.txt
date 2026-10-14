add_library(imu STATIC
    src/imu.c
    # Pico SDK bus access. Switches to a uT-Kernel version once the RTOS
    # build is set up.
    src/imu_bus_pico.c
)

target_include_directories(imu PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(imu PRIVATE shared)

if(BUILD_FIRMWARE)
    target_link_libraries(imu PRIVATE pico_stdlib)
endif()

target_link_libraries(imu PRIVATE hardware_i2c hardware_gpio)
