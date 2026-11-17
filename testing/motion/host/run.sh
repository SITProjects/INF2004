#!/bin/sh
# No hardware access: compile production motion logic with drive stubs.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
test_tmp=$(mktemp -d)
trap 'rm -rf "$test_tmp"' EXIT HUP INT TERM
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    "$repo_root/testing/motion/host/line_acquisition_test.c" \
    "$repo_root/app_program/motion/motion_move.c" \
    "$repo_root/app_program/motion/motion_pid.c" \
    -lm -o "$test_tmp/line-acquisition"
"$test_tmp/line-acquisition"

${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    -I"$repo_root/testing/motion/host/include" \
    "$repo_root/testing/motion/host/command_slot_test.c" \
    "$repo_root/app_program/motion/motion_command.c" \
    -o "$test_tmp/command-slot"
"$test_tmp/command-slot"
