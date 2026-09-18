cmake_minimum_required(VERSION 3.13)
project(obstacle_tests C)

enable_testing()
add_executable(obstacle_test
    obstacle_test.c
    ../../modules/obstacle/src/obstacle.c
)
target_include_directories(obstacle_test PRIVATE
    stubs
    ../../modules/obstacle/include
)
target_compile_features(obstacle_test PRIVATE c_std_99)
target_compile_options(obstacle_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME obstacle_regressions COMMAND obstacle_test)
