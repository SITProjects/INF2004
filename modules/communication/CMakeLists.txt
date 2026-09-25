add_library(communication STATIC
    src/connectivity.c
)

target_include_directories(communication PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(communication PRIVATE
    pico_stdlib
    wifi
    pico_time
)
