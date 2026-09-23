add_executable(wifi_connection
    src/wifi_connection.c
)

target_include_directories(wifi_connection
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(wifi_connection
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(wifi_connection 1)
pico_enable_stdio_uart(wifi_connection 0)

pico_add_extra_outputs(wifi_connection)
