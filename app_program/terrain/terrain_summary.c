/**
 * @file terrain_summary.c
 * @brief Run summary: hump count and heights, lifts and turns.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_summary.h"
#include <stddef.h>
#include "terrain_height.h"
#include "terrain_process.h"

/* Private types, constants, and macros. */

/* Word 0: hump count 4 | highest 8 | lifts 4 | turns 7 | spare 9. */
#define W_COUNTS 0u
#define HUMPS_SHIFT 0u
#define HUMPS_BITS 4u
#define HIGHEST_SHIFT 4u
#define HEIGHT_BITS 8u
#define LIFTS_SHIFT 12u
#define LIFTS_BITS 4u
#define TURNS_SHIFT 16u
#define TURNS_BITS 7u

/* Words 1 and 2: four 8-bit heights each, the earliest lowest. */
#define W_HEIGHTS 1u
#define HEIGHTS_PER_WORD 4u

/* Message: the low 3 bytes of word 0, then one byte per height. */
#define COUNT_BYTES 3u
#define BITS_PER_BYTE 8u
#define BYTE_MASK 0xFFu

_Static_assert(TERRAIN_SUMMARY_HUMP_COUNT_MAX == ((1u << HUMPS_BITS) - 1u),
               "hump count limit must match its field");
_Static_assert(TERRAIN_SUMMARY_LIFT_MAX == ((1u << LIFTS_BITS) - 1u),
               "lift limit must match its field");
_Static_assert(TERRAIN_SUMMARY_TURN_MAX == ((1u << TURNS_BITS) - 1u),
               "turn limit must match its field");
_Static_assert(TERRAIN_HEIGHT_MAX_MM == ((1u << HEIGHT_BITS) - 1u),
               "heights must fit their 8-bit fields");
_Static_assert((TURNS_SHIFT + TURNS_BITS) <= (COUNT_BYTES * BITS_PER_BYTE),
               "the counts must fit the message's first 3 bytes");
_Static_assert(TERRAIN_SUMMARY_HUMPS ==
                   ((TERRAIN_SUMMARY_WORDS - W_HEIGHTS) * HEIGHTS_PER_WORD),
               "the height words must hold every kept hump");
_Static_assert(TERRAIN_SUMMARY_BYTES == (COUNT_BYTES + TERRAIN_SUMMARY_HUMPS),
               "message is the counts then one byte per hump");

/* Static data. */

/* Private function declarations. */
static uint32_t count_get (terrain_summary_t const * p_summary, uint32_t shift,
                           uint32_t width);
static void count_add (terrain_summary_t * p_summary, uint32_t shift,
                       uint32_t width);

/* Public function definitions. */

void
terrain_summary_reset (terrain_summary_t * p_summary)
{
    if (p_summary == NULL)
    {
        return;
    }
    terrain_summary_t const empty = {{0u}};
    *p_summary = empty;
}

void
terrain_summary_add (terrain_summary_t * p_summary, terrain_event_type_t type,
                     uint32_t value)
{
    if (p_summary == NULL)
    {
        return;
    }

    if (type == TERRAIN_EVENT_LEVEL)
    {
        uint32_t const height =
            (value > TERRAIN_HEIGHT_MAX_MM) ? TERRAIN_HEIGHT_MAX_MM : value;

        /* The count before this hump is its slot. */
        uint32_t const index = count_get(p_summary, HUMPS_SHIFT, HUMPS_BITS);
        if (index < TERRAIN_SUMMARY_HUMPS)
        {
            uint32_t const word = W_HEIGHTS + (index / HEIGHTS_PER_WORD);
            uint32_t const shift = (index % HEIGHTS_PER_WORD) * HEIGHT_BITS;
            p_summary->word[word] = terrain_process_field_put(
                p_summary->word[word], shift, HEIGHT_BITS, height);
        }
        count_add(p_summary, HUMPS_SHIFT, HUMPS_BITS);

        if (height > count_get(p_summary, HIGHEST_SHIFT, HEIGHT_BITS))
        {
            p_summary->word[W_COUNTS] = terrain_process_field_put(
                p_summary->word[W_COUNTS], HIGHEST_SHIFT, HEIGHT_BITS, height);
        }
    }
    else if ((type == TERRAIN_EVENT_LIFTED) ||
             (type == TERRAIN_EVENT_OVERTURNED))
    {
        count_add(p_summary, LIFTS_SHIFT, LIFTS_BITS);
    }
    else if (type == TERRAIN_EVENT_TURNING)
    {
        count_add(p_summary, TURNS_SHIFT, TURNS_BITS);
    }
    else
    {
        /* Other kinds are not in the summary. */
    }
}

uint8_t
terrain_summary_hump_count (terrain_summary_t const * p_summary)
{
    return (uint8_t)count_get(p_summary, HUMPS_SHIFT, HUMPS_BITS);
}

uint8_t
terrain_summary_highest_mm (terrain_summary_t const * p_summary)
{
    return (uint8_t)count_get(p_summary, HIGHEST_SHIFT, HEIGHT_BITS);
}

uint8_t
terrain_summary_hump_mm (terrain_summary_t const * p_summary, uint32_t index)
{
    if ((p_summary == NULL) || (index >= TERRAIN_SUMMARY_HUMPS))
    {
        return 0u;
    }
    uint32_t const word = W_HEIGHTS + (index / HEIGHTS_PER_WORD);
    uint32_t const shift = (index % HEIGHTS_PER_WORD) * HEIGHT_BITS;
    return (uint8_t)terrain_process_field_get(p_summary->word[word], shift,
                                              HEIGHT_BITS);
}

uint8_t
terrain_summary_lifts (terrain_summary_t const * p_summary)
{
    return (uint8_t)count_get(p_summary, LIFTS_SHIFT, LIFTS_BITS);
}

uint8_t
terrain_summary_turns (terrain_summary_t const * p_summary)
{
    return (uint8_t)count_get(p_summary, TURNS_SHIFT, TURNS_BITS);
}

bool
terrain_summary_pack (terrain_summary_t const * p_summary, uint8_t * p_bytes)
{
    if ((p_summary == NULL) || (p_bytes == NULL))
    {
        return false;
    }

    /* Low byte first; bit 23 is spare and always 0. */
    uint32_t const counts = p_summary->word[W_COUNTS];
    for (uint32_t i = 0u; i < COUNT_BYTES; i++)
    {
        p_bytes[i] = (uint8_t)((counts >> (i * BITS_PER_BYTE)) & BYTE_MASK);
    }
    for (uint32_t i = 0u; i < TERRAIN_SUMMARY_HUMPS; i++)
    {
        p_bytes[COUNT_BYTES + i] = terrain_summary_hump_mm(p_summary, i);
    }
    return true;
}

/* Private function definitions. */

/* One field of word 0; 0 if p_summary is NULL. */
static uint32_t
count_get (terrain_summary_t const * p_summary, uint32_t shift, uint32_t width)
{
    if (p_summary == NULL)
    {
        return 0u;
    }
    return terrain_process_field_get(p_summary->word[W_COUNTS], shift, width);
}

/* Add one to a counter in word 0; it stops at its largest value. */
static void
count_add (terrain_summary_t * p_summary, uint32_t shift, uint32_t width)
{
    uint32_t const count =
        terrain_process_field_get(p_summary->word[W_COUNTS], shift, width);
    p_summary->word[W_COUNTS] = terrain_process_field_put(
        p_summary->word[W_COUNTS], shift, width,
        terrain_process_count_add_sat(count, 1u, width));
}
