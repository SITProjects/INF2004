add_library(telemetry STATIC
    src/telemetry.c
)

target_include_directories(telemetry PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(telemetry PRIVATE
    shared
    drivers
)
