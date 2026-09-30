add_library(motor STATIC
    src/motor.c
)

target_include_directories(motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motor PRIVATE shared)

target_link_libraries(motor PRIVATE pico_stdlib hardware_pwm hardware_clocks hardware_gpio)
