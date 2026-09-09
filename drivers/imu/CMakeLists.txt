add_library(imu STATIC
    src/imu.c
)

target_include_directories(imu PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(imu PRIVATE shared)

if(BUILD_FIRMWARE)
    target_link_libraries(imu PRIVATE pico_stdlib)
endif()
