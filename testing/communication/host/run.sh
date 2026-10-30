#!/bin/sh
# Compile real application code and lwIP headers; simulate external transport.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
sdk_path=${PICO_SDK_PATH:-/opt/pico-sdk}
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
cp "$repo_root/config/wifi_secrets.example.h" "$test_dir/wifi_secrets.h"
cc -std=c11 -Wall -Wextra -Werror -DTM_MQTT=1 \
    -I"$repo_root/testing/communication/host/include" \
    -I"$test_dir" -I"$repo_root/app_program/communication" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040/include" \
    -I"$sdk_path/lib/lwip/src/include" \
    "$repo_root/testing/communication/host/heartbeat_test.c" \
    "$repo_root/app_program/communication/mqtt_client.c" \
    -o "$test_dir/heartbeat_test"
"$test_dir/heartbeat_test"
