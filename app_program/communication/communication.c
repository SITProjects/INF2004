/**
 * @file communication.c
 * @brief Communication subsystem: starts and runs the Wi-Fi status task.
 *
 * The radio itself belongs to the kernel's service task; this task only
 * watches its status and reports changes (TEAM.md section 3).
 */

#include "communication.h"
#include "car_status.h"
#include "connectivity.h"
#if TM_MQTT
#include "telemetry_queue.h"
#endif

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
#if TM_MQTT
    return (ER)telemetry_queue_init();
#else
    return E_OK;
#endif
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
#if TM_MQTT && !APP_TEST && !IR_CALIBRATION_ONLY
        car_status_print();
#endif
        (void)tk_dly_tsk(POLL_PERIOD_MS);
    }
}
