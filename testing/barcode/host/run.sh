#!/bin/sh
# No hardware access: compile the production decoder and scan assembly.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
test_tmp=$(mktemp -d)
trap 'rm -rf "$test_tmp"' EXIT HUP INT TERM
${CC:-cc} -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    "$repo_root/testing/barcode/host/barcode_test.c" \
    "$repo_root/app_program/barcode/barcode_code39.c" \
    "$repo_root/app_program/barcode/barcode_scan.c" \
    -o "$test_tmp/barcode"
"$test_tmp/barcode"
