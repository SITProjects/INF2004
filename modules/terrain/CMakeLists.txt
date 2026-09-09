add_library(terrain STATIC
    src/terrain.c
)

target_include_directories(terrain PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(terrain PRIVATE
    shared
    drivers
)
