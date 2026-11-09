#!/bin/sh
# No hardware access: check the shared packed layouts in robot_types.h.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
test_tmp=$(mktemp -d)
trap 'rm -rf "$test_tmp"' EXIT HUP INT TERM
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    "$repo_root/testing/integration/host/packing_test.c" \
    -o "$test_tmp/packing"
"$test_tmp/packing"
