/**
 * @file communication.c
 * @brief Communication subsystem: starts and runs the Wi-Fi status task.
 *
 * The radio itself belongs to the kernel's service task; this task only
 * watches its status and reports changes (TEAM.md section 3).
 */

#include "communication.h"
#include "connectivity.h"

/* Private types, constants, and macros. */
#define POLL_PERIOD_MS 500

/* Static data. */
ID communication_tid_wifi;

/* Private function declarations. */
static void wifi_task (INT stacd, void * p_exinf);

/* Public function definitions. */

ER
communication_init (void)
{
    /* No kernel objects yet. */
    return E_OK;
}

ER
communication_start (void)
{
    communication_tid_wifi = app_start_task(wifi_task, COMMUNICATION_PRI_WIFI,
                                            0, "communication_task");
    return (communication_tid_wifi < E_OK) ? communication_tid_wifi : E_OK;
}

/* Private function definitions. */

static void
wifi_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    connectivity_init();
    for (;;)
    {
        connectivity_update();
        (void)tk_dly_tsk(POLL_PERIOD_MS);
    }
}
