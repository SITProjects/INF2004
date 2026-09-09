add_library(motor STATIC
    src/motor.c
)

target_include_directories(motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motor PRIVATE shared)

if(BUILD_FIRMWARE)
    target_link_libraries(motor PRIVATE pico_stdlib)
endif()
