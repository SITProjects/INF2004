add_library(line_follow STATIC EXCLUDE_FROM_ALL
    src/line_follow.c
)

target_include_directories(line_follow PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(line_follow PUBLIC line_sensor)
