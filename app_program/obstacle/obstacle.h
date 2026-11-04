/**
 * @file obstacle.h
 * @brief Obstacle subsystem: an ultrasonic sensor on a servo.
 *        GitHub: Kitsuneez.
 *
 * This header is the obstacle subsystem's PUBLIC interface: only what other
 * subsystems may use. Everything else stays private in the .c files. Other
 * subsystems include it as "../obstacle/obstacle.h".
 *
 * Other tasks read the obstacle state in two ways (IF-07). obstacle_flg_event
 * carries level bits (stop request, critical, bypass fault) and a RESULT
 * edge that one consumer clears. obstacle_get_state() copies the latest
 * packed result, its geometry, the stop request and the bypass state,
 * without waiting, from any task on either core. Only the scan task writes
 * either one.
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

/* Public includes. */
#include "../app.h"
#include "../robot_types.h"
#include "obstacle_config.h"
#include "obstacle_scan.h"
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** ID of the scan task, which owns the sensor, servo and GPIO interrupt. */
extern ID obstacle_tid_scan;

/** obstacle_flg_event (FLG_OBSTACLE): level, the stop request is active. */
#define OBSTACLE_EVT_STOP 0x01u
/** obstacle_flg_event (FLG_OBSTACLE): level, the critical flag is set. */
#define OBSTACLE_EVT_CRITICAL 0x02u
/** obstacle_flg_event (FLG_OBSTACLE): edge, a new result sequence; the one
 *  consumer clears it with tk_clr_flg(obstacle_flg_event, ~bit). */
#define OBSTACLE_EVT_RESULT 0x04u
/** obstacle_flg_event (FLG_OBSTACLE): level, the bypass is latched in FAULT
 *  and the motors stay inhibited until reboot. */
#define OBSTACLE_EVT_FAULT 0x08u

/** Event flag FLG_OBSTACLE; set and cleared only by the scan task. */
extern ID obstacle_flg_event;

/** Copy of the obstacle state, written once per scan-task loop. */
typedef struct
{
    robot_obstacle_result_t result;    /**< Packed status, edges, width. */
    obstacle_scan_geometry_t geometry; /**< Published with result.sequence. */
    bool b_stop_requested;             /**< Latched stop request. */
    uint8_t avoid_state;               /**< obstacle_avoid_state_t. */
    uint8_t fault_reason; /**< obstacle_avoid_fault_t, 0 if none. */
} obstacle_shared_state_t;

/* Public function declarations. */

/**
 * @brief Copy the latest obstacle state.
 * @param p_state Caller-owned copy to fill.
 * @return true with a consistent copy (all zeros before the first scan
 * loop); false for NULL, or if the scan task was writing on every try (only
 * possible from a task above it on the same core): try again later.
 * Any task, either core; never waits.
 */
bool obstacle_get_state (obstacle_shared_state_t * p_state);

#endif /* OBSTACLE_H */
