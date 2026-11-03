/**
 * @file telemetry_queue.h
 * @brief Bounded RAM outbox shared by application tasks and the radio task.
 */
#ifndef TELEMETRY_QUEUE_H
#define TELEMETRY_QUEUE_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */
#define TELEMETRY_QUEUE_CAPACITY 128u
#define TELEMETRY_TEXT_SIZE 64u

typedef struct
{
    uint32_t sequence;
    uint32_t captured_ms;
    char text[TELEMETRY_TEXT_SIZE];
} telemetry_record_t;

typedef enum
{
    TELEMETRY_QUEUED,
    TELEMETRY_BUSY,
    TELEMETRY_FULL,
    TELEMETRY_INVALID
} telemetry_enqueue_result_t;

/* Public function declarations. */
/** @brief Create the mutex once during communication_init(), before producers.
 * @return Zero on success, negative kernel error on failure. No queue reset.
 */
int telemetry_queue_init (void);

/** @brief Copy a nonempty NUL-terminated text (at most 63 bytes) into RAM.
 * @param captured_ms Capture uptime, independent of network state.
 * @param p_text Text copied before return; never a pointer retained by queue.
 * @param p_sequence Optional output for the assigned sequence on QUEUED/FULL.
 * @return QUEUED, BUSY (retry same sample), FULL (newest rejected), or INVALID.
 * @note Task context on either core; never blocks. FULL consumes a sequence,
 * making dropped records visible as gaps. IDs identify records within a boot.
 * RAM survives network loss only. Caller must handle BUSY/FULL explicitly.
 */
telemetry_enqueue_result_t telemetry_queue_push (uint32_t captured_ms,
                                                 const char * p_text,
                                                 uint32_t * p_sequence);

/** @brief Radio task only: copy oldest record without removing it.
 * @return True when copied; false if empty, invalid output, or mutex busy.
 */
bool telemetry_queue_peek (telemetry_record_t * p_record);

/** @brief Radio task only: remove matching oldest record after broker PUBACK.
 * @return True if removed; false if mutex busy, empty, or sequence mismatched.
 * @note Retry on contention. No network call is made while holding the mutex.
 */
bool telemetry_queue_ack (uint32_t sequence);

#endif /* TELEMETRY_QUEUE_H */
