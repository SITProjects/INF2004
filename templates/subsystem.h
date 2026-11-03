/**
 * @file subsystem.h
 * @brief TODO: Subsystem -- describe what it does. GitHub: <username>.
 *
 * This header is the subsystem's PUBLIC interface: only what other
 * subsystems may use (object IDs, message formats, priorities). Everything
 * else stays private in the .c files. Other subsystems include it as
 * "../subsystem/subsystem.h".
 */

#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

/* Public includes. */
#include "../app.h"

/* Public types, constants, and macros. */

/**
 * Main task priority: this task's row in TEAM.md section 4. A new task
 * needs a row agreed with the team first; 22 is only a placeholder below
 * every listed task.
 */
#define SUBSYSTEM_PRI_MAIN 22

/* Public function declarations. */

#endif /* SUBSYSTEM_H */
