/**
 * @file subsystem.c
 * @brief TODO: Subsystem -- creates its kernel objects and starts its
 *        tasks.
 *
 * subsystem_init():  create this subsystem's kernel objects (semaphores,
 *                    mailboxes, ...). Do not start tasks here.
 * subsystem_start(): create and start this subsystem's tasks.
 *
 * Drivers for this subsystem's hardware go in its folder as
 * drv_<name>.c / drv_<name>.h, called only from the task that owns the
 * peripheral (TEAM.md section 3). Record every pin, task, object and
 * interface in TEAM.md, and declare subsystem_init(), subsystem_start()
 * and subsystem_test() in app.h.
 */

#include "subsystem.h"

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */
static void main_task (INT stacd, void * p_exinf);

/* Public function definitions. */

ER
subsystem_init (void)
{
    return E_OK;
}

ER
subsystem_start (void)
{
    ID tid = app_start_task(main_task, SUBSYSTEM_PRI_MAIN, 0, "subsystem_task");
    return (tid < E_OK) ? tid : E_OK;
}

/* Private function definitions. */

static void
main_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        /* TODO: The task's work; block or delay, never busy-wait. */
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
