add_library(line_sensor STATIC EXCLUDE_FROM_ALL
    src/line_sensor.c
)

target_include_directories(line_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(line_sensor PRIVATE
    hardware_adc
)
