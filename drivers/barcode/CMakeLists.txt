add_library(barcode STATIC
    src/barcode.c
)

target_include_directories(barcode PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(barcode PRIVATE shared)

if(BUILD_FIRMWARE)
    target_link_libraries(barcode PRIVATE pico_stdlib)
endif()
