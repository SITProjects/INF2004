/**
 * @file terrain_summary.h
 * @brief Run summary: hump count and heights, collisions and turns,
 *        counted from the events as they are made.
 *
 * Pure C with no kernel calls, so it can be tested on a PC. The summary
 * has no lock of its own: its owner (terrain_monitor.c) holds the
 * platform lock around every call that touches the shared one.
 *
 * Summary words, masks and shifts, bit 0 first; all zero is an empty
 * summary:
 *   word 0: hump count 4 | highest 8 | collisions 4 | turns 7 | spare 9
 *   word 1: heights of humps 1-4, 8 bits each, the first in bits 0-7
 *   word 2: heights of humps 5-8, the same way
 *
 * Packed message, TERRAIN_SUMMARY_BYTES bytes, little-endian:
 *   bytes 0-2: word 0 bits 0-23 (bit 23 is spare, always 0)
 *   bytes 3-10: heights of humps 1-8, mm, one byte each (0 if unused)
 *
 * Internal to the terrain module; other modules should use
 * terrain_monitor.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_SUMMARY_H
#define TERRAIN_SUMMARY_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include "terrain_monitor.h"

/* Public types, constants, and macros. */

/* Largest counts the fields hold; counts stop there. */
#define TERRAIN_SUMMARY_HUMP_COUNT_MAX 15u
#define TERRAIN_SUMMARY_COLLISION_MAX 15u
#define TERRAIN_SUMMARY_TURN_MAX 127u

/* Public function declarations. */

/**
 * @brief Empty a summary: no humps, collisions or turns.
 * @param p_summary Summary; ignored if NULL.
 */
void terrain_summary_reset (terrain_summary_t * p_summary);

/**
 * @brief Count one event.
 * @param p_summary Summary; ignored if NULL.
 * @param type Kind of event. LEVEL counts a hump, COLLISION a collision
 *        and TURNING a turn; other kinds change nothing.
 * @param value For LEVEL the hump's height, mm; heights over 255 are
 *        stored as 255. Ignored for other kinds.
 * Counts stop at their largest value. The first TERRAIN_SUMMARY_HUMPS
 * humps keep their height; later ones are counted and can still raise the
 * highest hump, but their height is not kept.
 */
void terrain_summary_add (terrain_summary_t * p_summary,
                          terrain_event_type_t type, uint32_t value);

/**
 * @brief Humps finished (LEVEL events).
 * @param p_summary Summary.
 * @return 0 to TERRAIN_SUMMARY_HUMP_COUNT_MAX; 0 if NULL.
 */
uint8_t terrain_summary_hump_count (terrain_summary_t const * p_summary);

/**
 * @brief Highest hump.
 * @param p_summary Summary.
 * @return mm, 0 to 255; 0 before the first hump or if NULL.
 */
uint8_t terrain_summary_highest_mm (terrain_summary_t const * p_summary);

/**
 * @brief Height of one hump.
 * @param p_summary Summary.
 * @param index 0 for the first hump, up to TERRAIN_SUMMARY_HUMPS - 1.
 * @return mm, 0 to 255; 0 for a hump not yet finished, an index out of
 *         range, or NULL.
 */
uint8_t terrain_summary_hump_mm (terrain_summary_t const * p_summary,
                                 uint32_t index);

/**
 * @brief Collisions (COLLISION events).
 * @param p_summary Summary.
 * @return 0 to TERRAIN_SUMMARY_COLLISION_MAX; 0 if NULL.
 */
uint8_t terrain_summary_collisions (terrain_summary_t const * p_summary);

/**
 * @brief Turns (TURNING events).
 * @param p_summary Summary.
 * @return 0 to TERRAIN_SUMMARY_TURN_MAX; 0 if NULL.
 */
uint8_t terrain_summary_turns (terrain_summary_t const * p_summary);

/**
 * @brief Pack a summary into the message layout above.
 * @param p_summary Summary.
 * @param p_bytes Receives TERRAIN_SUMMARY_BYTES bytes.
 * @return True if packed; false, with @p p_bytes unchanged, if either
 *         pointer is NULL.
 */
bool terrain_summary_pack (terrain_summary_t const * p_summary,
                           uint8_t * p_bytes);

#endif /* TERRAIN_SUMMARY_H */
