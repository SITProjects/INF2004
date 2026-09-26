add_library(wifi STATIC
    src/wifi.c
)

target_include_directories(wifi PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(wifi PUBLIC
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)
