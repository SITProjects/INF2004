/**
 * @file motion.c
 * @brief Initialize motors and start the line-command integration task.
 *
 * The motion subsystem owns PWM slices 4 and 5 (GP8-GP11, TEAM.md
 * section 3). Startup initializes them braked, then transfers ownership
 * to the task implemented in motion_command.c.
 */

#include "motion.h"
#include "motion_drive.h"
#include "motion_command.h"

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */

/* Public function definitions. */

ER
motion_init (void)
{
    return motion_command_init();
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
    return motion_command_start();
}

/* Private function definitions. */
