# MQTT telemetry hardware check

The active client is `app_program/communication/mqtt_client.c`, running in
`cyw43_utk_task()` on core 0. It publishes queued obstacle telemetry only;
there are no application heartbeat messages or topic subscriptions.
The MQTT protocol keepalive remains enabled (60 seconds).

## Configuration and build

Set Wi-Fi and MQTT settings in the gitignored `config/wifi_secrets.h`.
Use a reachable numeric IPv4 broker address and a unique client ID.
Do not commit credentials. The client uses plain TCP without broker login.

Start the lab broker:

```sh
mosquitto -c testing/communication/mosquitto-test.conf -v
```

Subscribe using your configured MQTT_TOPIC followed by `/telemetry`:

```sh
mosquitto_sub -h 127.0.0.1 -p 1883 -t mtk3smp-pico/heartbeat/telemetry -v
```

Build and flash the generated UF2:

```sh
make -C build_make MQTT=1 TEST=1 -j8
```

Normal obstacle scanning must be enabled; calibration mode skips samples.

## Expected evidence

After connection, expect samples about once per second, for example:

```text
seq=0 captured_ms=12000 obstacle flags=17 edges=255 width_mm=0
```

Check that the base MQTT_TOPIC receives no `hb` messages for at least
15 seconds. The `/heartbeat/` text remains in the configured topic prefix
for compatibility; only the `/telemetry` topic is published.

Telemetry uses QoS 1 without retain. The FIFO holds 128 records in RAM,
removes records after PUBACK, and retains pending records across network
loss. Replay may duplicate messages. Full queues reject newest samples;
reboot loses queued data. Stop/restart the broker and disable/restore the
access point; verify replay preserves sequence numbers and capture times.
Reconnect attempts are throttled by 3000 ms after failure.

Record actual serial and subscriber evidence in TEST_LOG.md. A firmware
build or simulated host test does not verify hardware delivery.
COM-U07's old inbound logging case is retired; no topic is subscribed.

## Host regression checks

```sh
sh testing/communication/host/run.sh
```

These exercise the real application client with simulated transport:
no heartbeat publishes/subscriptions, reconnect retries, Wi-Fi loss,
queue capacity, contention, replay, PUBACK handling, and clock wrap.
They use fake configuration and never read local credentials.
