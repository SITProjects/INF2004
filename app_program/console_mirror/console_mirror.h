/**
 * @file console_mirror.h
 * @brief Copy every console line to MQTT, for runs without a USB cable.
 *
 * Built only with MQTT=1. The makefile links the console output through
 * this module (-Wl,--wrap=tm_snd_dat), so every tm_printf() line still
 * reaches the USB console and is also kept, stamped with the uptime in
 * milliseconds, in a RAM buffer. console_mirror_mqtt.c sends the buffer
 * from the radio task in batches of whole lines to MQTT_TOPIC "/console".
 * Lines printed before the broker connects wait in the buffer; when it is
 * full, whole lines are dropped and a "console_mirror: dropped lines"
 * line reports the total. The radio task's own "telemetry: TX", "ACK" and
 * "queued" lines are not sent, so the mirror never feeds on itself.
 *
 * This half uses the kernel; the lwIP half is console_mirror_mqtt.c (the
 * two headers cannot share a file).
 */

#ifndef CONSOLE_MIRROR_H
#define CONSOLE_MIRROR_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** Largest batch console_mirror_take() writes, in bytes. */
#define CONSOLE_MIRROR_BATCH_MAX 576u

/* Public function declarations. */

#if TM_MQTT
/**
 * @brief Move the next whole lines out of the buffer.
 * @param p_out Receives up to CONSOLE_MIRROR_BATCH_MAX bytes, not
 *        terminated: whole lines, each ending in '\n', or one piece of a
 *        line longer than a batch.
 * @return Bytes written; 0 when no finished line is waiting.
 * @note Radio task only (one consumer). The lines leave the buffer, so
 *       the caller keeps them until they are sent.
 */
uint32_t console_mirror_take (char * p_out);

/** @brief Record that a batch reached the broker. Radio task only. */
void console_mirror_mark_online (void);

/**
 * @brief Wait until the mirror has sent a batch to the broker.
 * @param timeout_ms Longest wait, in milliseconds.
 * @return True once a batch has gone out, false on timeout.
 * @note Task context; sleeps in 100 ms steps. Lets a test run start only
 *       when its output can reach MQTT.
 */
bool console_mirror_wait_online (uint32_t timeout_ms);
#endif /* TM_MQTT */

#endif /* CONSOLE_MIRROR_H */
