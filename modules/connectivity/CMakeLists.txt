add_library(connectivity STATIC
    src/connectivity.c
)

target_include_directories(connectivity PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(connectivity PRIVATE
    wifi
    pico_stdlib
)
