#!/bin/sh
# Compile real application code and lwIP headers; simulate external transport.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
sdk_path=${PICO_SDK_PATH:-/opt/pico-sdk}
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
cc -std=c11 -Wall -Wextra -Werror -DTM_MQTT=1 \
    -I"$repo_root/testing/communication/host/include" \
    -I"$test_dir" -I"$repo_root/app_program/communication" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040/include" \
    -I"$sdk_path/lib/lwip/src/include" \
    "$repo_root/testing/communication/host/heartbeat_test.c" \
    "$repo_root/app_program/communication/mqtt_client.c" \
    "$repo_root/app_program/communication/telemetry_queue.c" \
    "$repo_root/testing/communication/host/queue_port_stub.c" \
    -o "$test_dir/heartbeat_test"
"$test_dir/heartbeat_test"

cc -std=c11 -Wall -Wextra -Werror -DTM_MQTT=1 \
    -I"$repo_root/testing/communication/host/include" \
    -I"$repo_root/app_program/communication" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040/include" \
    -I"$sdk_path/lib/lwip/src/include" \
    "$repo_root/testing/communication/host/telemetry_test.c" \
    "$repo_root/app_program/communication/mqtt_client.c" \
    "$repo_root/app_program/communication/telemetry_queue.c" \
    -o "$test_dir/telemetry_test"
"$test_dir/telemetry_test"
