add_library(gpio STATIC
    src/gpio.c
)

target_include_directories(gpio PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(gpio PRIVATE shared)

if(BUILD_FIRMWARE)
    target_link_libraries(gpio PRIVATE pico_stdlib)
endif()
