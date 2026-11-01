/**
 * @file drv_encoder.h
 * @brief Quadrature wheel encoder driver counting in PIO0 state machines.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */
typedef enum
{
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_COUNT
} encoder_id_t;

/* Public function declarations. */

/**
 * @brief Configure one quadrature encoder and start counting from zero.
 * @param id Encoder to configure.
 * @param pin_a GPIO connected to phase A.
 * @param pin_b GPIO connected to phase B; must be pin_a + 1.
 * @param b_inverted Negate the count for a wheel mounted in mirror image.
 * @return True on success; false for an invalid or already initialised
 * encoder, invalid pins, or (first call) a PIO0 state machine that is
 * already running another program.
 * Counts every edge of A and B (four per encoder cycle) in hardware, with
 * no CPU interrupts. The first call loads the counting program into PIO0
 * instruction slots 0-23; encoder @p id runs on state machine 2 + @p id.
 * Those slots, state machines 2 and 3, and both pins belong to this driver
 * afterwards. Call from one task, before any encoder_get_count().
 */
bool encoder_init (encoder_id_t id, uint32_t pin_a, uint32_t pin_b,
                   bool b_inverted);

/**
 * @brief Read the signed edge count since initialisation, modulo 2^16.
 * @return Count, positive when A leads B (after any inversion), or 0 for
 * an invalid or uninitialised encoder.
 * The count wraps from 32767 to -32768 (about 65 m of travel at 2 mm per
 * count), so use only differences between two reads, taken as
 * (int16_t)(now - prev); they are correct across a wrap while the wheel
 * moves fewer than 32768 counts between reads.
 * Reading empties the state machine's FIFO, so call from one task only.
 * Busy-waits well under 1 us for a fresh sample.
 */
int16_t encoder_get_count (encoder_id_t id);

#endif /* DRV_ENCODER_H */
