/**
 * @file telemetry_queue_port.c
 * @brief Nonblocking mutex access for the telemetry outbox on either core.
 */
#include "telemetry_queue_port.h"
#include <tk/tkernel.h>

/* Static data. */
static ID queue_mutex;

/* Public function definitions. */
int
telemetry_queue_port_init (void)
{
    T_CMTX params = {.mtxatr = TA_INHERIT};
    ID created = tk_cre_mtx(&params);
    if (created < E_OK)
    {
        return (int)created;
    }
    /* Radio service may already be running: publish ID after creation. */
    __atomic_store_n(&queue_mutex, created, __ATOMIC_RELEASE);
    return 0;
}

bool
telemetry_queue_port_lock (void)
{
    ID current = __atomic_load_n(&queue_mutex, __ATOMIC_ACQUIRE);
    return (current > 0) && (tk_loc_mtx(current, TMO_POL) == E_OK);
}

void
telemetry_queue_port_unlock (void)
{
    ID current = __atomic_load_n(&queue_mutex, __ATOMIC_RELAXED);
    (void)tk_unl_mtx(current);
}
