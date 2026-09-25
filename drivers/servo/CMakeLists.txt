add_library(servo STATIC
    src/servo.c
)

target_include_directories(servo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(servo PRIVATE shared)

target_link_libraries(servo PRIVATE pico_stdlib hardware_pwm hardware_clocks hardware_gpio)
