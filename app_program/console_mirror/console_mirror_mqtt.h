/**
 * @file console_mirror_mqtt.h
 * @brief Send the console mirror's lines to MQTT (lwIP half).
 *
 * See console_mirror.h. Batches go to MQTT_TOPIC "/console" at QoS 0; a
 * batch is kept until lwIP reports it sent, and sent again after a
 * reconnect, so a line may arrive twice but is not lost while the buffer
 * has room.
 */

#ifndef CONSOLE_MIRROR_MQTT_H
#define CONSOLE_MIRROR_MQTT_H

/* Public includes. */

/* Public function declarations. */

#if TM_MQTT
struct mqtt_client_s;

/**
 * @brief Send the next batch of console lines while the broker is
 *        connected.
 * @param p_client The radio task's MQTT client, or NULL before it exists.
 * @note Radio task only (lwIP core context), called on every service
 *       pass. Never prints: a print here would be mirrored again.
 */
void console_mirror_mqtt_service (struct mqtt_client_s * p_client);
#endif /* TM_MQTT */

#endif /* CONSOLE_MIRROR_MQTT_H */
