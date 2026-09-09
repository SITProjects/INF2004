add_library(motion STATIC
    src/motion.c
)

target_include_directories(motion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motion PRIVATE
    shared
    drivers
)
