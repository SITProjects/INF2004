/**
 * @file terrain_event.c
 * @brief Terrain events: the packed event word and a queue of events.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_event.h"
#include <stdbool.h>
#include <stddef.h>
#include "terrain_process.h"

/* Private types, constants, and macros. */

/* Event word: type 4 | value 12 | time 16. */
#define TYPE_SHIFT 0u
#define TYPE_BITS 4u
#define VALUE_SHIFT 4u
#define TIME_SHIFT 16u
#define TIME_BITS 16u

/* Largest value code, 4095. */
#define VALUE_MAX ((1u << TERRAIN_EVENT_VALUE_BITS) - 1u)

/* Bookkeeping word: head 4 | count 5 | dropped 8 | spare 15. */
#define HEAD_SHIFT 0u
#define HEAD_BITS 4u
#define COUNT_SHIFT 4u
#define COUNT_BITS 5u
#define DROPPED_SHIFT 9u
#define DROPPED_BITS 8u

_Static_assert(sizeof(terrain_event_t) == 4u, "event must be one word");
_Static_assert(TERRAIN_EVENT_TYPES <= (1u << TYPE_BITS),
               "event types must fit the type field");
_Static_assert(TERRAIN_EVENT_QUEUE_LEN == (1u << HEAD_BITS),
               "queue length must match the head field");
_Static_assert(TERRAIN_EVENT_QUEUE_LEN < (1u << COUNT_BITS),
               "a full queue's count must fit the count field");

/* Static data. */

/* Private function declarations. */
static bool type_known (uint32_t code);
static uint32_t book_get (terrain_event_queue_t const * p_queue, uint32_t shift,
                          uint32_t width);
static void book_put (terrain_event_queue_t * p_queue, uint32_t shift,
                      uint32_t width, uint32_t value);

/* Public function definitions. */

terrain_event_t
terrain_event_make (terrain_event_type_t type, uint32_t value_code,
                    uint32_t time_10ms)
{
    terrain_event_t event = {0u};
    uint32_t const code = (uint32_t)type;
    if ((code == (uint32_t)TERRAIN_EVENT_NONE) || !type_known(code))
    {
        return event;
    }

    /* Clamp, never wrap: a value past the field reads as its limit. */
    uint32_t const value = (value_code > VALUE_MAX) ? VALUE_MAX : value_code;

    uint32_t word = 0u;
    word = terrain_process_field_put(word, TYPE_SHIFT, TYPE_BITS, code);
    word = terrain_process_field_put(word, VALUE_SHIFT,
                                     TERRAIN_EVENT_VALUE_BITS, value);
    /* 10 ms ticks, wrapping: readers only use differences. */
    word = terrain_process_field_put(word, TIME_SHIFT, TIME_BITS, time_10ms);
    event.word = word;
    return event;
}

terrain_event_type_t
terrain_event_type (terrain_event_t event)
{
    uint32_t const code =
        terrain_process_field_get(event.word, TYPE_SHIFT, TYPE_BITS);
    if (!type_known(code))
    {
        return TERRAIN_EVENT_NONE;
    }
    return (terrain_event_type_t)code;
}

int32_t
terrain_event_value (terrain_event_t event)
{
    uint32_t const code = terrain_process_field_get(event.word, VALUE_SHIFT,
                                                    TERRAIN_EVENT_VALUE_BITS);

    switch (terrain_event_type(event))
    {
        case TERRAIN_EVENT_NONE:
            return 0;
        case TERRAIN_EVENT_ASCENDING:
        case TERRAIN_EVENT_DESCENDING:
            /* Pitch, two's complement. */
            return terrain_process_decode_signed(code,
                                                 TERRAIN_EVENT_VALUE_BITS);
        default:
            return (int32_t)code;
    }
}

uint16_t
terrain_event_time_10ms (terrain_event_t event)
{
    if (terrain_event_type(event) == TERRAIN_EVENT_NONE)
    {
        return 0u;
    }
    return (uint16_t)terrain_process_field_get(event.word, TIME_SHIFT,
                                               TIME_BITS);
}

void
terrain_event_queue_reset (terrain_event_queue_t * p_queue)
{
    if (p_queue == NULL)
    {
        return;
    }

    terrain_event_queue_t const empty = {0};
    *p_queue = empty;
}

void
terrain_event_queue_push (terrain_event_queue_t * p_queue,
                          terrain_event_t event)
{
    if ((p_queue == NULL) || (terrain_event_type(event) == TERRAIN_EVENT_NONE))
    {
        return;
    }

    uint32_t const head = book_get(p_queue, HEAD_SHIFT, HEAD_BITS);
    uint32_t const count = book_get(p_queue, COUNT_SHIFT, COUNT_BITS);

    if (count >= TERRAIN_EVENT_QUEUE_LEN)
    {
        /* Full: the new event takes the oldest one's slot. */
        p_queue->slot[head] = event;
        book_put(p_queue, HEAD_SHIFT, HEAD_BITS,
                 (head + 1u) % TERRAIN_EVENT_QUEUE_LEN);
        book_put(p_queue, COUNT_SHIFT, COUNT_BITS, TERRAIN_EVENT_QUEUE_LEN);
        uint32_t const dropped = book_get(p_queue, DROPPED_SHIFT, DROPPED_BITS);
        book_put(p_queue, DROPPED_SHIFT, DROPPED_BITS,
                 terrain_process_count_add_sat(dropped, 1u, DROPPED_BITS));
        return;
    }

    p_queue->slot[(head + count) % TERRAIN_EVENT_QUEUE_LEN] = event;
    book_put(p_queue, COUNT_SHIFT, COUNT_BITS, count + 1u);
}

terrain_event_t
terrain_event_queue_pop (terrain_event_queue_t * p_queue)
{
    terrain_event_t const none = {0u};
    if (p_queue == NULL)
    {
        return none;
    }

    uint32_t const count = book_get(p_queue, COUNT_SHIFT, COUNT_BITS);
    if (count == 0u)
    {
        return none;
    }

    uint32_t const head = book_get(p_queue, HEAD_SHIFT, HEAD_BITS);
    terrain_event_t const event = p_queue->slot[head];
    book_put(p_queue, HEAD_SHIFT, HEAD_BITS,
             (head + 1u) % TERRAIN_EVENT_QUEUE_LEN);
    book_put(p_queue, COUNT_SHIFT, COUNT_BITS, count - 1u);
    return event;
}

uint32_t
terrain_event_queue_count (terrain_event_queue_t const * p_queue)
{
    if (p_queue == NULL)
    {
        return 0u;
    }
    return book_get(p_queue, COUNT_SHIFT, COUNT_BITS);
}

uint32_t
terrain_event_queue_dropped (terrain_event_queue_t const * p_queue)
{
    if (p_queue == NULL)
    {
        return 0u;
    }
    return book_get(p_queue, DROPPED_SHIFT, DROPPED_BITS);
}

/* Private function definitions. */

static bool
type_known (uint32_t code)
{
    return code < TERRAIN_EVENT_TYPES;
}

static uint32_t
book_get (terrain_event_queue_t const * p_queue, uint32_t shift, uint32_t width)
{
    return terrain_process_field_get(p_queue->book, shift, width);
}

static void
book_put (terrain_event_queue_t * p_queue, uint32_t shift, uint32_t width,
          uint32_t value)
{
    p_queue->book =
        terrain_process_field_put(p_queue->book, shift, width, value);
}
