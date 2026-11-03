/* Fake host-test configuration. Never reads local Wi-Fi credentials. */
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H
#define WIFI_SSID "host-test-only"
#define WIFI_PASSWORD "not-a-secret"
#define MQTT_BROKER_IP "192.0.2.1"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "host-test-pico"
#define MQTT_TOPIC "mtk3smp-pico/heartbeat"
#endif
