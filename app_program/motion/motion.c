/**
 * @file motion.c
 * @brief Initialize motors, encoders and the line-command integration task.
 *
 * The motion subsystem owns PWM slices 4 and 5 (GP8-GP11) and PIO0
 * (encoders: left on GP13/GP14, right on GP2/GP3; TEAM.md section 3).
 * Startup initializes the motors and encoders, then transfers motor
 * ownership to the task implemented in motion_command.c.
 */

#include "motion.h"
#include "motion_drive.h"
#include "motion_move.h"
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
    if ((motion_drive_init() != MOTION_OK) || (motion_move_init() != MOTION_OK))
    {
        tm_printf(
            (UB *)"motion: failed to initialize the motors or encoders\n");
        return E_SYS;
    }
    tm_printf((UB *)"motion: motors initialized and braked\n");
    tm_printf((UB *)"motion: encoders counting on PIO0\n");
    return motion_command_start();
}

/* Private function definitions. */
