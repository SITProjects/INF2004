# MQTT heartbeat hardware check

The active client is `app_program/communication/mqtt_client.c`. It runs in
`cyw43_utk_task()` on core 0; the communication task must not call it.
`MQTT=1` selects this client and upstream lwIP MQTT, leaving the old
`lib/liblwip/.../mqtt_utk.c` demo unlinked. No lib/liblwip files are changed.
There are no new tasks, queues, or kernel objects for heartbeat-only use.

## Configuration

Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` if absent.
Set WIFI_SSID and WIFI_PASSWORD, then MQTT_BROKER_IP, MQTT_BROKER_PORT,
MQTT_CLIENT_ID, and MQTT_TOPIC. The broker address must be a numeric IPv4
address reachable from the Pico, not localhost. Use a unique client ID per
Pico. The configuration is gitignored; never commit actual credentials.

This client uses plain TCP, with no TLS or broker username/password. Use a
broker that permits this on your isolated lab network. Wi-Fi authentication
and broker authentication are separate.

## Broker and firmware

With Mosquitto and its command-line clients installed, run from repo root
on the broker computer (or adjust the configuration path):

```sh
mosquitto -c testing/communication/mosquitto-test.conf -v
```

Permit inbound TCP port 1883 through its firewall. The supplied listener
accepts anonymous LAN clients; do not expose it to the Internet. If using a
remote dev container, the broker must still be reachable from the Pico.

In another broker-computer terminal, substitute your configured topic:

```sh
mosquitto_sub -h 127.0.0.1 -p 1883 -t mtk3smp-pico/heartbeat -v
```

Build from the repository and flash the generated MQTT/test UF2:

```sh
cd build_make
make MQTT=1 TEST=1 -j8
```

Open the Pico USB serial console. Existing COM-U01/U02 tests check radio and
Wi-Fi readiness, not MQTT delivery. MQTT hardware cases COM-U04 through
COM-U07 in TEST_LOG.md are manual and require a broker.

## Expected evidence

After `mqtt: connected to <broker-ip>`, wait five seconds:

```text
mqtt: TX [mtk3smp-pico/heartbeat] "hb 0"
mqtt: RX [mtk3smp-pico/heartbeat] "hb 0"
```

Expect increasing counters about every five seconds, also visible in
mosquitto_sub. TX means lwIP accepted the publish request; matching RX and
an independent subscriber provide broker delivery evidence. QoS is 0 and
retain is disabled, so interruptions may lose messages. The first heartbeat
occurs five seconds after each MQTT connection is established.

Test the inbound path from the broker computer:

```sh
mosquitto_pub -h 127.0.0.1 -p 1883 -t mtk3smp-pico/heartbeat -m hello-pico
```

The Pico should print an RX line containing hello-pico. Incoming messages
are only logged; they do not control the vehicle.

Stop/restart the broker, then disable/restore the access point. Detection
may take TCP/MQTT timeouts, followed by a 3000 ms MQTT retry delay. Heartbeats
must resume without resetting the board or multiplying their frequency.
The counter continues across reconnects, resets on reboot, and advances
when lwIP accepts a publish. Allocation and immediate connect failures are
also retried with the same delay. Subscription failures trigger a deferred
disconnect and reconnect after this delay to restore the RX check. A failed publish is logged and retried
with the same counter at the next heartbeat interval.

For COM-U07, use a payload of at least 128 bytes, then a short payload.
RX payload logging holds at most 127 bytes and sanitizes nonprintable
payload bytes; fragmented messages are logged after their last fragment.

Save serial and broker/subscriber output under testing/communication/logs/.
Record each actual run in TEST_LOG.md with commit, tester, duration, and
result. Do not record a build or simulated transport test as hardware PASS.

`make -j8` leaves MQTT disabled by default. No heartbeat publishes in that
profile, or with WIFI=none/NET=none.

## Host behavior regression check

With a host C compiler and Pico SDK lwIP sources available, run:

```sh
sh testing/communication/host/run.sh
```

This compiles the actual application client against real lwIP headers and a
simulated external MQTT transport. It checks heartbeat timing, retry delays,
subscription failure recovery, Wi-Fi loss, fragmented/oversized/empty RX,
publish errors, and clock wraparound. It uses the example configuration and
does not read local credentials. It does not exercise real TCP, Wi-Fi, or a
broker and does not replace the hardware cases above.
