#!/bin/sh
# No hardware access: check the bypass context's packed state.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
test_tmp=$(mktemp -d)
trap 'rm -rf "$test_tmp"' EXIT HUP INT TERM
obstacle="$repo_root/app_program/obstacle"
# No -Werror: obstacle_config.h's preprocessor checks warn under host GCC.
${CC:-cc} -std=c11 -Wall -Wextra \
    -fsanitize=address,undefined -g \
    -I"$obstacle" -I"$repo_root/app_program" -I"$repo_root/config" \
    -I"$repo_root/include" \
    "$repo_root/testing/obstacle/host/avoid_state_test.c" \
    "$obstacle/obstacle_width.c" "$obstacle/obstacle_decision.c" \
    -lm -o "$test_tmp/avoid-state"
"$test_tmp/avoid-state"
