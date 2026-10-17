/**
 * @file line_follow.h
 * @brief Line-following subsystem: two IR sensors. GitHub: kaiwenteoo.
 *
 * This header is the line-following subsystem's PUBLIC interface: only what
 * other subsystems may use. Everything else stays private in the .c files.
 * Other subsystems include it as "../line_follow/line_follow.h".
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

/* Public includes. */
#include "../app.h"

/* Public types, constants, and macros. */

/** Line task priority; the line-following band is 16-17 (TEAM.md). */
#define LINE_FOLLOW_PRI_SENSE 16

/** ID of the line task, which owns the ADC. */
extern ID line_follow_tid_sense;

/* Public function declarations. */

#endif /* LINE_FOLLOW_H */
