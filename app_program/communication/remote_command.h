/**
 * @file remote_command.h
 * @brief Commands from the dashboard: run state and barcode actions.
 *
 * The radio task hands each message on the command topic to
 * remote_command_handle(), and other tasks read the results lock-free:
 * the run state is one flag and each table entry one aligned word, so no
 * reader ever waits and no lwIP callback takes a kernel object. Messages
 * are text: "start", "stop", "clear", or "map <barcode> <action>", with an
 * action of STRAIGHT, LEFT, RIGHT or UTURN.
 *
 * The module is pure: no kernel calls, so host tests can drive it.
 */

#ifndef REMOTE_COMMAND_H
#define REMOTE_COMMAND_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include "../line_follow/line_track.h"

/* Public types, constants, and macros. */

/** Longest barcode text a table entry holds. */
#define REMOTE_TEXT_MAX 3u

/** Barcodes the table holds. */
#define REMOTE_MAP_SIZE 8u

/** @brief What became of a message. */
typedef enum
{
    REMOTE_COMMAND_OK = 0, /**< Done. */
    REMOTE_COMMAND_BAD,    /**< Not a command this module knows. */
    REMOTE_COMMAND_FULL    /**< The barcode table has no free entry. */
} remote_command_result_t;

/* Public function declarations. */

/**
 * @brief Carry out one message.
 * @param p_text Message text, not necessarily terminated.
 * @param length Characters in @p p_text.
 * @return See remote_command_result_t.
 * Radio task only.
 */
remote_command_result_t remote_command_handle (char const * p_text,
                                               uint32_t length);

/**
 * @brief Whether the dashboard has the car stopped.
 * @return True until "start" in MQTT builds, which wait for the dashboard;
 *         false in builds with no dashboard to start them.
 * Any task; never waits.
 */
bool remote_command_stopped (void);

/**
 * @brief The action set for a barcode.
 * @param p_text Decoded barcode text, terminated.
 * @return Its action, or LINE_JUNCTION_DEFAULT_TURN if it has none.
 * Any task; never waits.
 */
line_turn_t remote_command_turn (char const * p_text);

/**
 * @brief Whether the dashboard has set an action for a barcode.
 * @param p_text Decoded barcode text, terminated.
 * @return True if @p p_text is in the table, STRAIGHT included.
 * Any task; never waits.
 */
bool remote_command_mapped (char const * p_text);

#endif /* REMOTE_COMMAND_H */
