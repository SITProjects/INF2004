/**
 * @file line_follow.h
 * @brief Line-following subsystem: two digital IR sensors. GitHub: kaiwenteoo.
 *
 * This header is the line-following subsystem's PUBLIC interface: only what
 * other subsystems may use. Everything else stays private in the .c files.
 * Other subsystems include it as "../line_follow/line_follow.h".
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

/* Public includes. */
#include "../app.h"
#include "line_track.h"

/* Public types, constants, and macros. */

/** Line task priority: tLine, design freeze 4 + 3 (TEAM.md section 4). */
#define LINE_FOLLOW_PRI_SENSE 7

/** ID of the line task, which owns the line DO pins (GP26, GP27). */
extern ID line_follow_tid_sense;

/* Public function declarations. */

/**
 * @brief Set what the car does at the next cross junction.
 * @param turn LINE_TURN_STRAIGHT (the default), LEFT, RIGHT or UTURN.
 * @note Any task, any core (one word, read by the line task). The
 *       instruction is used by the next junction only, then returns to
 *       LINE_JUNCTION_DEFAULT_TURN (STRAIGHT); a later call replaces an
 *       unused one. Called by the
 *       barcode subsystem when it decodes a barcode.
 */
void line_follow_set_turn (line_turn_t turn);

#endif /* LINE_FOLLOW_H */
