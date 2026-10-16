add_subdirectory(barcode)
add_subdirectory(gpio)
add_subdirectory(imu)
add_subdirectory(ir_sensor)
add_subdirectory(motor)
add_subdirectory(servo)
add_subdirectory(ultrasonic)
add_subdirectory(wifi)

add_library(drivers INTERFACE)

target_link_libraries(drivers INTERFACE
    barcode
    gpio
    imu
    ir_sensor
    motor
    servo
    ultrasonic
    wifi
)
