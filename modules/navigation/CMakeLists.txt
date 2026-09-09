add_library(navigation STATIC
    src/navigation.c
)

target_include_directories(navigation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(navigation PRIVATE
    shared
    drivers
)
