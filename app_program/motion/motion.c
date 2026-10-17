/**
 * @file motion.c
 * @brief Motion subsystem: configures the motors and holds them braked.
 *
 * The motion subsystem owns PWM slices 4 and 5 (GP8-GP11, TEAM.md
 * section 3). There is no motion task yet, so nothing drives the motors
 * after start-up; a task that does must take over ownership of those
 * slices.
 */

#include "motion.h"
#include "motion_drive.h"

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
    if (motion_drive_init() != MOTION_OK)
    {
        tm_printf((UB *)"motion: failed to initialize the motors\n");
        return E_SYS;
    }
    tm_printf((UB *)"motion: motors initialized and braked\n");
    return E_OK;
}

/* Private function definitions. */
