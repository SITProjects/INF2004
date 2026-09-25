add_library(obstacle STATIC
    src/obstacle.c
)

target_include_directories(obstacle PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(obstacle PRIVATE
    servo
    ultrasonic
    hardware_gpio
    pico_stdlib
)
