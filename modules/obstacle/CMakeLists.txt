add_library(obstacle STATIC
    src/ranging.c
    src/servo.c
    src/obstacle.c
)

target_include_directories(obstacle PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(obstacle PRIVATE
    hardware_pwm
    hardware_clocks
    pico_stdlib
)
