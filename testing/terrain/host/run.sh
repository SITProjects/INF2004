#!/bin/sh
# No hardware access: compile the production compass logic with a stubbed
# magnetometer.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
test_tmp=$(mktemp -d)
trap 'rm -rf "$test_tmp"' EXIT HUP INT TERM
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    "$repo_root/testing/terrain/host/compass_test.c" \
    "$repo_root/app_program/terrain/terrain_compass.c" \
    -lm -o "$test_tmp/compass"
"$test_tmp/compass"
