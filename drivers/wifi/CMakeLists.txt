add_library(wifi STATIC
    src/wifi.c
)

target_include_directories(wifi PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(wifi PRIVATE shared)

if(BUILD_FIRMWARE)
    target_link_libraries(wifi PRIVATE pico_stdlib)
endif()
