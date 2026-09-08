add_library(ultrasonic STATIC
    src/ultrasonic.c
)

target_include_directories(ultrasonic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ultrasonic PRIVATE shared)

if(BUILD_FIRMWARE)
    target_link_libraries(ultrasonic PRIVATE pico_stdlib)
endif()
