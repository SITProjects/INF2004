#!/bin/sh
# No hardware access: compile the production line tracking logic.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
test_tmp=$(mktemp -d)
trap 'rm -rf "$test_tmp"' EXIT HUP INT TERM
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    "$repo_root/testing/line_follow/host/line_track_test.c" \
    "$repo_root/app_program/line_follow/line_track.c" \
    -lm -o "$test_tmp/line-track"
"$test_tmp/line-track"
