add_library(terrain STATIC
    src/terrain.c
    src/terrain_process.c
    src/terrain_log.c
    # Pico SDK platform layer. Switches to a uT-Kernel version once the
    # RTOS build is set up.
    src/terrain_platform.c
)

target_include_directories(terrain PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(terrain PRIVATE
    shared
    drivers
    pico_stdlib
    hardware_sync
)
