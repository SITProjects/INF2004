/**
 * @file motion.c
 * @brief Motion subsystem: configures the motors and holds them braked,
 * and starts the wheel encoders counting.
 *
 * The motion subsystem owns PWM slices 4 and 5 (GP8-GP11) and PIO0
 * (encoders on GP2/GP3 and GP13/GP14, TEAM.md section 3). There is no
 * motion task yet, so nothing calls motion_move_update() or drives the
 * motors after start-up; a task that does must take over ownership of
 * them.
 */

#include "motion.h"
#include "motion_drive.h"
#include "motion_move.h"

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */

/* Public function definitions. */

ER
motion_init (void)
{
    /* No kernel objects yet. */
    return E_OK;
}

ER
motion_start (void)
{
    if ((motion_drive_init() != MOTION_OK) || (motion_move_init() != MOTION_OK))
    {
        tm_printf(
            (UB *)"motion: failed to initialize the motors or encoders\n");
        return E_SYS;
    }
    tm_printf((UB *)"motion: motors initialized and braked\n");
    tm_printf((UB *)"motion: encoders counting on PIO0\n");
    return E_OK;
}

/* Private function definitions. */
