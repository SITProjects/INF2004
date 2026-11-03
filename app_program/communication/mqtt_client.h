/**
 * @file mqtt_client.h
 * @brief Application telemetry service, executed by the kernel radio task.
 */
#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

/* Public includes. */
#include <stdbool.h>

/* Public function declarations. */

/**
 * @brief Maintain MQTT and send queued telemetry when connected.
 * @param b_network_up True when Wi-Fi has an IPv4 address.
 * @note Built with MQTT=1. Call repeatedly, only from cyw43_utk_task() on
 *       core 0, after lwIP initialization. First call initializes the client;
 *       communication_init() creates the queue mutex before producers start.
 *       The client needs no separate init. Owns all client state
 *       and callbacks. Never call from application tasks or another core.
 *       Returns promptly; failed connections are retried after 3000 ms.
 *       Uses MQTT_* settings in wifi_secrets.h.
 *       Drains telemetry_queue on MQTT_TOPIC "/telemetry" using QoS 1,
 *       with one outstanding record. PUBACK permits deferred FIFO removal.
 *       Failed or interrupted attempts retain the record and may duplicate
 *       delivery. No retain flag, power-loss persistence, or subscriber ACK.
 */
void mqtt_client_service (bool b_network_up);

#endif /* MQTT_CLIENT_H */
