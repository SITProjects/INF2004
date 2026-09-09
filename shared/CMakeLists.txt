add_library(shared STATIC
    src/shared.c
)

target_include_directories(shared PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
