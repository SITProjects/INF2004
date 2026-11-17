#!/bin/sh
# No hardware access: compile the production line tracking logic.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
test_tmp=$(mktemp -d)
trap 'rm -rf "$test_tmp"' EXIT HUP INT TERM

${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    "$repo_root/testing/line_follow/host/line_track_marks_test.c" \
    -lm -o "$test_tmp/line-track-marks"
"$test_tmp/line-track-marks"

# Every scenario again from either side of the 16-bit distance mark wrap.
for offset in 0 32700 65500 -300; do
    ${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
        -DTRAVEL_OFFSET_MM="$offset" \
        "$repo_root/testing/line_follow/host/line_track_test.c" \
        "$repo_root/app_program/line_follow/line_track.c" \
        -lm -o "$test_tmp/line-track"
    "$test_tmp/line-track"
done
