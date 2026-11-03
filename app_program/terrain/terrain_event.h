/**
 * @file terrain_event.h
 * @brief Terrain events: the packed event word and a queue of events.
 *
 * Pure C with no kernel calls, so it can be tested on a PC. The queue has
 * no lock of its own: its owner (terrain_monitor.c) holds the platform
 * lock around every call that touches a shared queue.
 *
 * Event word, masks and shifts, bit 0 first:
 *   type 4 (bits 0-3) | value 12 (bits 4-15) | time 16 (bits 16-31)
 * Queue bookkeeping word:
 *   head 4 (oldest slot) | count 5 (0-16) | dropped 8 (saturating) |
 *   spare 15
 *
 * Internal to the terrain module; other modules should use
 * terrain_monitor.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_EVENT_H
#define TERRAIN_EVENT_H

/* Public includes. */
#include <stdint.h>
#include "terrain_monitor.h"

/* Public types, constants, and macros. */

/* Events the queue holds; a power of two that fits the 4-bit head. */
#define TERRAIN_EVENT_QUEUE_LEN 16u

/* Width of the event value field, bits. */
#define TERRAIN_EVENT_VALUE_BITS 12u

/** @brief A queue of events; the oldest is dropped when it is full. */
typedef struct
{
    terrain_event_t slot[TERRAIN_EVENT_QUEUE_LEN]; /**< Ring of events. */
    uint32_t book; /**< Packed head, count and dropped count. */
} terrain_event_queue_t;

/* Public function declarations. */

/**
 * @brief Pack an event.
 * @param type Kind of event; a code of TERRAIN_EVENT_TYPES or more gives
 *        a NONE event.
 * @param value_code Value field: for ASCENDING and DESCENDING a code from
 *        terrain_process_encode_signed() with TERRAIN_EVENT_VALUE_BITS;
 *        otherwise the value itself. Codes over 4095 are stored as 4095.
 * @param time_10ms Time in 10 ms ticks; only the low 16 bits are kept.
 * @return The event; all zero (NONE) for type NONE or an unknown type.
 */
terrain_event_t terrain_event_make (terrain_event_type_t type,
                                    uint32_t value_code, uint32_t time_10ms);

/**
 * @brief Kind of a packed event.
 * @param event Event.
 * @return The kind; TERRAIN_EVENT_NONE for an unknown code.
 */
terrain_event_type_t terrain_event_type (terrain_event_t event);

/**
 * @brief Value of a packed event in its kind's unit.
 * @param event Event.
 * @return Signed (-2048 to 2047) for ASCENDING and DESCENDING, 0 to 4095
 *         for other kinds, 0 for NONE or an unknown kind.
 */
int32_t terrain_event_value (terrain_event_t event);

/**
 * @brief Time of a packed event.
 * @param event Event.
 * @return 10 ms ticks, 0 to 65535; 0 for NONE or an unknown kind.
 */
uint16_t terrain_event_time_10ms (terrain_event_t event);

/**
 * @brief Empty a queue and clear its dropped count.
 * @param p_queue Queue; ignored if NULL.
 */
void terrain_event_queue_reset (terrain_event_queue_t * p_queue);

/**
 * @brief Add an event at the end of a queue.
 * @param p_queue Queue.
 * @param event Event; NONE and unknown kinds are not added.
 * If the queue is full, the oldest event is dropped and the dropped count
 * goes up by one, stopping at 255. Does nothing if @p p_queue is NULL.
 */
void terrain_event_queue_push (terrain_event_queue_t * p_queue,
                               terrain_event_t event);

/**
 * @brief Remove and return the oldest event.
 * @param p_queue Queue.
 * @return The event; NONE if the queue is empty or @p p_queue is NULL.
 */
terrain_event_t terrain_event_queue_pop (terrain_event_queue_t * p_queue);

/**
 * @brief Events waiting in a queue.
 * @param p_queue Queue.
 * @return 0 to TERRAIN_EVENT_QUEUE_LEN; 0 if @p p_queue is NULL.
 */
uint32_t terrain_event_queue_count (terrain_event_queue_t const * p_queue);

/**
 * @brief Events dropped from a full queue since its last reset.
 * @param p_queue Queue.
 * @return 0 to 255, stopping at 255; 0 if @p p_queue is NULL.
 */
uint32_t terrain_event_queue_dropped (terrain_event_queue_t const * p_queue);

#endif /* TERRAIN_EVENT_H */
