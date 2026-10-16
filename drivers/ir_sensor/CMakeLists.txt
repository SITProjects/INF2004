add_library(ir_sensor STATIC EXCLUDE_FROM_ALL
    src/ir_sensor.c
)

target_include_directories(ir_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ir_sensor PRIVATE
    hardware_adc
)
