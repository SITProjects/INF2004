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
    "$repo_root/testing/communication/host/connection_test.c" \
    "$repo_root/app_program/communication/mqtt_client.c" \
    "$repo_root/app_program/communication/remote_command.c" \
    "$repo_root/app_program/line_follow/line_track.c" \
    "$repo_root/app_program/communication/telemetry_queue.c" \
    "$repo_root/testing/communication/host/queue_port_stub.c" \
    -o "$test_dir/connection_test"
"$test_dir/connection_test"

cc -std=c11 -Wall -Wextra -Werror -DTM_MQTT=1 \
    -I"$repo_root/testing/communication/host/include" \
    -I"$repo_root/app_program/communication" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040/include" \
    -I"$sdk_path/lib/lwip/src/include" \
    "$repo_root/testing/communication/host/telemetry_test.c" \
    "$repo_root/app_program/communication/mqtt_client.c" \
    "$repo_root/app_program/communication/remote_command.c" \
    "$repo_root/app_program/line_follow/line_track.c" \
    "$repo_root/app_program/communication/telemetry_queue.c" \
    -o "$test_dir/telemetry_test"
"$test_dir/telemetry_test"

cc -std=c11 -Wall -Wextra -Werror -DTM_MQTT=1 \
    -fsanitize=address,undefined -g \
    -I"$repo_root/testing/communication/host/include" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040" \
    -I"$repo_root/lib/liblwip/sysdepend/pico_rp2040/include" \
    -I"$sdk_path/lib/lwip/src/include" \
    "$repo_root/testing/communication/host/console_mirror_test.c" \
    "$repo_root/app_program/console_mirror/console_mirror_mqtt.c" \
    -o "$test_dir/console_mirror_test"
"$test_dir/console_mirror_test"

cc -std=c11 -Wall -Wextra -Werror -DTM_MQTT=1 \
    -fsanitize=address,undefined -g \
    "$repo_root/testing/communication/host/remote_command_test.c" \
    "$repo_root/app_program/communication/remote_command.c" \
    "$repo_root/app_program/line_follow/line_track.c" \
    -o "$test_dir/remote_command_test"
"$test_dir/remote_command_test"
