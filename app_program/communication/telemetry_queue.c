/**
 * @file telemetry_queue.c
 * @brief Fixed FIFO; no allocation, networking, or waiting in critical sections.
 */
#include "telemetry_queue.h"
#include "telemetry_queue_port.h"
#include <string.h>

/* Static data. */
static telemetry_record_t records[TELEMETRY_QUEUE_CAPACITY];
static uint16_t head;
static uint16_t count;
static uint32_t next_sequence;

/* Public function definitions. */
int
telemetry_queue_init (void)
{
    return telemetry_queue_port_init();
}

telemetry_enqueue_result_t
telemetry_queue_push (uint32_t captured_ms, const char * p_text,
                      uint32_t * p_sequence)
{
    if (p_text == NULL)
    {
        return TELEMETRY_INVALID;
    }
    uint16_t len = 0u;
    while ((len < TELEMETRY_TEXT_SIZE) && (p_text[len] != '\0'))
    {
        ++len;
    }
    if ((len == 0u) || (len == TELEMETRY_TEXT_SIZE))
    {
        return TELEMETRY_INVALID;
    }
    if (!telemetry_queue_port_lock())
    {
        return TELEMETRY_BUSY;
    }
    uint32_t sequence = next_sequence++;
    if (p_sequence != NULL)
    {
        *p_sequence = sequence;
    }
    if (count == TELEMETRY_QUEUE_CAPACITY)
    {
        telemetry_queue_port_unlock();
        return TELEMETRY_FULL;
    }
    uint16_t tail = (uint16_t)((head + count) % TELEMETRY_QUEUE_CAPACITY);
    records[tail].sequence = sequence;
    records[tail].captured_ms = captured_ms;
    memcpy(records[tail].text, p_text, len + 1u);
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
    bool b_matches = (count != 0u) && (records[head].sequence == sequence);
    if (b_matches)
    {
        head = (uint16_t)((head + 1u) % TELEMETRY_QUEUE_CAPACITY);
        --count;
    }
    telemetry_queue_port_unlock();
    return b_matches;
}
