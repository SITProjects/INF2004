/**
 * @file telemetry_queue.c
 * @brief Fixed FIFO; no allocation, networking, or waiting in critical sections.
 */
#include "telemetry_queue.h"
#include "telemetry_queue_port.h"
#include <stddef.h>

/* Static data. */
static telemetry_record_t records[TELEMETRY_QUEUE_CAPACITY];
static uint8_t head;
static uint8_t count;
static uint8_t next_sequence;

_Static_assert(TELEMETRY_QUEUE_CAPACITY < 256u,
               "Sequences must tell every queued record apart");

/* Public function definitions. */
int
telemetry_queue_init (void)
{
    return telemetry_queue_port_init();
}

telemetry_enqueue_result_t
telemetry_queue_push (uint32_t captured_ms, robot_telemetry_payload_t payload,
                      uint32_t * p_sequence)
{
    if ((payload.type < (uint32_t)ROBOT_TELEMETRY_OBSTACLE) ||
        (payload.type > (uint32_t)ROBOT_TELEMETRY_BARCODE))
    {
        return TELEMETRY_INVALID;
    }
    if (!telemetry_queue_port_lock())
    {
        return TELEMETRY_BUSY;
    }
    uint32_t const sequence = next_sequence;
    next_sequence = (uint8_t)(next_sequence + 1u);
    if (p_sequence != NULL)
    {
        *p_sequence = sequence;
    }
    if (count == TELEMETRY_QUEUE_CAPACITY)
    {
        telemetry_queue_port_unlock();
        return TELEMETRY_FULL;
    }
    uint8_t const tail = (uint8_t)((head + count) % TELEMETRY_QUEUE_CAPACITY);
    uint32_t word = robot_field_put(0u, ROBOT_TELEMETRY_TYPE_SHIFT,
                                    ROBOT_TELEMETRY_TYPE_BITS, payload.type);
    word = robot_field_put(word, ROBOT_TELEMETRY_SEQUENCE_SHIFT,
                           ROBOT_TELEMETRY_SEQUENCE_BITS, sequence);
    uint32_t const ticks = captured_ms / ROBOT_TELEMETRY_TICK_MS;
    records[tail].words[0] =
        robot_field_put(word, ROBOT_TELEMETRY_TIME_SHIFT,
                        ROBOT_TELEMETRY_TIME_BITS, ticks & 0xFFFFFu);
    records[tail].words[1] = payload.first;
    records[tail].words[2] = payload.second;
    ++count;
    telemetry_queue_port_unlock();
    return TELEMETRY_QUEUED;
}

bool
telemetry_queue_peek (telemetry_record_t * p_record)
{
    if ((p_record == NULL) || !telemetry_queue_port_lock())
    {
        return false;
    }
    bool b_available = count != 0u;
    if (b_available)
    {
        *p_record = records[head];
    }
    telemetry_queue_port_unlock();
    return b_available;
}

bool
telemetry_queue_ack (uint32_t sequence)
{
    if (!telemetry_queue_port_lock())
    {
        return false;
    }
    bool const b_matches =
        (count != 0u) &&
        (robot_field_get(records[head].words[0], ROBOT_TELEMETRY_SEQUENCE_SHIFT,
                         ROBOT_TELEMETRY_SEQUENCE_BITS) == sequence);
    if (b_matches)
    {
        head = (uint8_t)((head + 1u) % TELEMETRY_QUEUE_CAPACITY);
        --count;
    }
    telemetry_queue_port_unlock();
    return b_matches;
}
