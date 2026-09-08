set(APP_SOURCES
    main.c
    vehicle_controller/vehicle_controller.c
)

if(BUILD_FIRMWARE)
    add_executable(vehicle_controller ${APP_SOURCES})
else()
    # Compile the skeleton without requiring a main() implementation.
    add_library(vehicle_controller OBJECT ${APP_SOURCES})
endif()

target_include_directories(vehicle_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/vehicle_controller
)

target_link_libraries(vehicle_controller PRIVATE
    shared
    motion
    navigation
    obstacle
    telemetry
    terrain
)

if(BUILD_FIRMWARE)
    target_link_libraries(vehicle_controller PRIVATE pico_stdlib)
    set_target_properties(vehicle_controller PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    pico_enable_stdio_usb(vehicle_controller 1)
    pico_enable_stdio_uart(vehicle_controller 0)
    pico_add_extra_outputs(vehicle_controller)
endif()
