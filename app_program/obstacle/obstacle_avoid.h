/**
 * @file obstacle_avoid.h
 * @brief Hardware-free bypass planning and corridor checks for obstacles.
 *
 * The bypass turns by an angle toward the gap, drives diagonally out, turns
 * back parallel to the line, verifies the corridor ahead, passes the
 * obstacle, turns toward the line, drives back to it with stop-at-line,
 * turns to the original heading and follows. The turn angle is as shallow
 * as the room in front of the obstacle allows, never below angle_min_deg;
 * at 90 degrees the path is a box.
 *
 * Every function works only on its arguments: no hardware, kernel calls or
 * shared state, so they may run on either core and obstacle_test.c drives
 * them directly. Distances are millimetres. Ranges are from the servo pivot
 * (obstacle_width_range_mm()); lateral offsets are + left. Turns are in
 * place about the axle midpoint, so the pass distance measured before the
 * sidestep still holds after it.
 *
 * obstacle_avoid_step() is the bypass state machine. The scan task calls it
 * once per loop with everything it read (scan result, motion status, events
 * and odometry, and the results of the last loop's actions) and carries out
 * the returned actions in the documented order. It never blocks.
 */

#ifndef OBSTACLE_AVOID_H
#define OBSTACLE_AVOID_H

/* Public includes. */
#include "../robot_types.h"
#include "obstacle_scan.h"
#include "obstacle_width.h"
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** Car dimensions and bypass limits; production passes the config macros. */
typedef struct
{
    uint16_t half_width_mm;    /**< Half the car's width, W / 2. */
    uint16_t margin_mm;        /**< Clearance kept from the obstacle. */
    uint16_t axle_to_pivot_mm; /**< Axle midpoint to servo pivot, forward. */
    uint16_t rear_mm;          /**< Axle midpoint to rear bumper. */
    uint16_t depth_mm;         /**< Assumed obstacle depth (not measured). */
    uint16_t sidestep_min_mm;  /**< Smallest first sidestep. */
    uint16_t sidestep_max_mm;  /**< Largest first sidestep. */
    uint16_t lateral_max_mm;   /**< Largest total offset from the line. */
    uint16_t pass_max_mm;      /**< Largest pass distance accepted. */
    uint16_t return_extra_mm;  /**< Drive past the offset to find the line. */
    uint16_t overshoot_mm;     /**< IR row to axle; 0 skips that leg. */
    uint8_t angle_min_deg;     /**< Shallowest turn, 5..90 degrees. */
} obstacle_avoid_geometry_t;

/** Why obstacle_avoid_plan() accepted or refused a result. */
typedef enum
{
    OBSTACLE_AVOID_PLAN_OK,         /**< A bypass is possible. */
    OBSTACLE_AVOID_PLAN_NOT_PATH,   /**< No side to try: see the plan. */
    OBSTACLE_AVOID_PLAN_NO_GEOMETRY /**< No centre distance published. */
} obstacle_avoid_plan_status_t;

/** A planned bypass; the distances are valid only when status is OK. */
typedef struct
{
    obstacle_avoid_plan_status_t status;
    bool b_gap_right;     /**< Bypass on the right (DIR_RIGHT_120). */
    bool b_edge_unknown;  /**< Gap-side edge not found: sidestep is maximal. */
    bool b_limited;       /**< Sidestep or pass capped at its maximum. */
    uint16_t sidestep_mm; /**< First sideways offset (not distance driven). */
    uint16_t pass_mm;     /**< Axle travel along the line, from the stop. */
    uint8_t angle_deg;    /**< Turn angle of every turn, 5..90 degrees. */
} obstacle_avoid_plan_t;

/** Result of checking a corridor sweep against the path ahead. */
typedef enum
{
    OBSTACLE_AVOID_CORRIDOR_CLEAR,  /**< Nothing in the path ahead. */
    OBSTACLE_AVOID_CORRIDOR_SHIFT,  /**< Sidestep the given extra first. */
    OBSTACLE_AVOID_CORRIDOR_BLOCKED /**< Invalid data, or no shift fits. */
} obstacle_avoid_corridor_t;

/** Timeouts and motion speeds; production passes the config macros. */
typedef struct
{
    uint32_t stop_ack_ms;      /**< INHIBIT set or cleared to its report. */
    uint32_t result_ms;        /**< Stop to a side result. */
    uint32_t prepare_ms;       /**< Rearm to a fresh clear result. */
    uint32_t send_ms;          /**< Command pending to queued. */
    uint32_t verify_ms;        /**< Corridor request to its result. */
    uint32_t block_wait_ms;    /**< Off the line, held, before FAULT. */
    uint32_t retry_delay_ms;   /**< Held before the one retry. */
    uint16_t move_speed_mm_s;  /**< MOTION_MOVE_SPEED_MM_S, nonzero. */
    uint16_t turn_speed_deg_s; /**< MOTION_TURN_SPEED_DEG_S, nonzero. */
    uint16_t resume_min_mm;    /**< Smaller remainders count as done. */
    uint8_t resume_min_deg;    /**< Smaller turn remainders count as done. */
    uint8_t max_sidesteps;     /**< First sidestep plus extra ones, >= 1. */
} obstacle_avoid_timing_t;

/** Everything the state machine needs that does not change. */
typedef struct
{
    obstacle_avoid_geometry_t geometry;
    obstacle_avoid_timing_t timing;
    obstacle_width_config_t width; /**< For the corridor check. */
} obstacle_avoid_config_t;

/** States of the bypass state machine. */
typedef enum
{
    OBSTACLE_AVOID_IDLE,         /**< Following; watching for a stop. */
    OBSTACLE_AVOID_STOPPING,     /**< INHIBIT set; waiting for INHIBITED. */
    OBSTACLE_AVOID_WAIT_RESULT,  /**< On the line, held; sweep running. */
    OBSTACLE_AVOID_WAIT_CLEAR,   /**< On the line, held; no timeout. */
    OBSTACLE_AVOID_RELEASING,    /**< INHIBIT cleared; waiting to leave it. */
    OBSTACLE_AVOID_PREPARE,      /**< Rearmed; waiting for a fresh CLEAR. */
    OBSTACLE_AVOID_SEND,         /**< Command pending until queued. */
    OBSTACLE_AVOID_RUN_LEG,      /**< Command queued; waiting for its event. */
    OBSTACLE_AVOID_VERIFY,       /**< Corridor sweep after a sidestep. */
    OBSTACLE_AVOID_BLOCKED_WAIT, /**< Off the line, held; path blocked. */
    OBSTACLE_AVOID_RETRY_WAIT,   /**< Off the line, held; before the retry. */
    OBSTACLE_AVOID_FAULT         /**< Latched until reboot; INHIBIT kept. */
} obstacle_avoid_state_t;

/** Why the state machine latched FAULT. */
typedef enum
{
    OBSTACLE_AVOID_FAULT_NONE,      /**< Not in FAULT. */
    OBSTACLE_AVOID_FAULT_STOP_ACK,  /**< INHIBITED never reported or left. */
    OBSTACLE_AVOID_FAULT_KERNEL,    /**< INHIBIT, event clear or send. */
    OBSTACLE_AVOID_FAULT_ENCODER,   /**< MOTION_EVT_ENC_FAULT. */
    OBSTACLE_AVOID_FAULT_REJECTED,  /**< MOTION_EVT_REJECTED. */
    OBSTACLE_AVOID_FAULT_ODOMETRY,  /**< No consistent odometry. */
    OBSTACLE_AVOID_FAULT_SCANNER,   /**< Fresh result or corridor late. */
    OBSTACLE_AVOID_FAULT_BLOCKED,   /**< Off the line, blocked too long. */
    OBSTACLE_AVOID_FAULT_RETRY_USED /**< A second retryable fault. */
} obstacle_avoid_fault_t;

/** Outcome of the last loop's command send. */
typedef enum
{
    OBSTACLE_AVOID_SEND_NONE, /**< Nothing was sent. */
    OBSTACLE_AVOID_SEND_OK,   /**< Queued. */
    OBSTACLE_AVOID_SEND_BUSY, /**< E_TMOUT: buffer full; still pending. */
    OBSTACLE_AVOID_SEND_ERROR /**< Any other error, or events not cleared. */
} obstacle_avoid_send_t;

/** Outcome of the last loop's corridor request. */
typedef enum
{
    OBSTACLE_AVOID_CORRIDOR_REQ_NONE,    /**< Not requested. */
    OBSTACLE_AVOID_CORRIDOR_REQ_STARTED, /**< Sweep started. */
    OBSTACLE_AVOID_CORRIDOR_REQ_REFUSED  /**< Another sweep was running. */
} obstacle_avoid_corridor_req_t;

/** Everything read this loop, plus the results of last loop's actions. */
typedef struct
{
    uint32_t now_ms;                   /**< Kernel time in milliseconds. */
    robot_obstacle_result_t scan;      /**< obstacle_scan_update(). */
    bool b_stop_requested;             /**< obstacle_scan_stop_requested(). */
    obstacle_scan_geometry_t geometry; /**< obstacle_scan_geometry(). */
    obstacle_scan_corridor_t corridor; /**< obstacle_scan_corridor(). */
    robot_motion_mode_t motion_mode;   /**< motion_get_status(). */
    robot_motion_done_t motion_done;   /**< motion_get_status(). */
    bool b_move_done;      /**< MOTION_EVT_MOVE_DONE, read before the status. */
    bool b_stall;          /**< MOTION_EVT_STALL. */
    bool b_enc_fault;      /**< MOTION_EVT_ENC_FAULT. */
    bool b_rejected;       /**< MOTION_EVT_REJECTED. */
    bool b_odometry_valid; /**< motion_get_odometry() result. */
    int32_t distance_mm;   /**< Odometry distance. */
    int32_t heading_deg;   /**< Odometry total turn. */
    bool b_inhibit_failed; /**< Last set or clear failed. */
    obstacle_avoid_send_t send; /**< Last send. */
    bool b_rearmed;             /**< Last loop called rearm. */
    uint8_t rearm_sequence;     /**< Its return value. */
    obstacle_avoid_corridor_req_t corridor_request; /**< Last request. */
} obstacle_avoid_input_t;

/**
 * Actions for one loop, carried out in this order: set INHIBIT, clear
 * INHIBIT, rearm the scanner, request a corridor sweep, then clear every
 * MOTION_EVT_ bit and, only if that worked, send the command.
 */
typedef struct
{
    bool b_set_inhibit;
    bool b_clear_inhibit;
    bool b_rearm;
    bool b_corridor;
    bool b_send;                 /**< Clear the motion events, then send cmd. */
    uint32_t cmd;                /**< MBF_MOTION_CMD word when b_send. */
    obstacle_avoid_state_t from; /**< State before this step. */
    obstacle_avoid_state_t to;   /**< State after this step. */
    char const * p_reason;       /**< Static text when the state changed. */
} obstacle_avoid_action_t;

/** Bypass legs, in order; VERIFY is a step without a command. */
typedef enum
{
    OBSTACLE_AVOID_LEG_TURN_OUT,  /**< Turn 90 degrees toward the gap. */
    OBSTACLE_AVOID_LEG_SIDESTEP,  /**< Move sideways off the line. */
    OBSTACLE_AVOID_LEG_TURN_BACK, /**< Turn back parallel to the line. */
    OBSTACLE_AVOID_LEG_VERIFY,    /**< Corridor sweep. */
    OBSTACLE_AVOID_LEG_PASS,      /**< Move past the obstacle. */
    OBSTACLE_AVOID_LEG_TURN_IN,   /**< Turn toward the line. */
    OBSTACLE_AVOID_LEG_RETURN,    /**< Move back, stopping at the line. */
    OBSTACLE_AVOID_LEG_OVERSHOOT, /**< Put the axle on the line. */
    OBSTACLE_AVOID_LEG_TURN_HOME, /**< Original heading. */
    OBSTACLE_AVOID_LEG_FOLLOW     /**< Resume line following. */
} obstacle_avoid_leg_t;

/** State machine context; fields are private to obstacle_avoid.c. */
typedef struct
{
    obstacle_avoid_config_t const * p_config;
    obstacle_avoid_state_t state;
    obstacle_avoid_fault_t fault;
    uint32_t entered_ms; /* Time the current state was entered. */
    obstacle_avoid_plan_t plan;
    bool b_plan_sequence;  /* plan_sequence holds a used result. */
    uint8_t plan_sequence; /* Result the current plan came from. */
    obstacle_avoid_leg_t leg;
    bool b_leg_started;   /* remaining holds this leg's remainder. */
    uint32_t remaining;   /* Of the current leg, mm or degrees. */
    uint32_t part;        /* Of the command in flight, mm or degrees. */
    uint32_t cmd;         /* Pending or running command. */
    bool b_to_follow;     /* RELEASING ends with FOLLOW. */
    bool b_need_prepare;  /* Rearm before the next translate. */
    uint16_t sidestep_mm; /* Current sidestep target. */
    uint32_t lateral_mm;  /* Total sideways offset completed. */
    uint32_t forward_mm;  /* Progress along the line from diagonals. */
    uint8_t sidesteps;    /* Sidesteps completed. */
    bool b_retry_used;
    bool b_blocked; /* blocked_since_ms is running. */
    uint32_t blocked_since_ms;
    bool b_rearm_known;
    uint8_t rearm_sequence;
    bool b_corridor_started;
    uint8_t corridor_sequence; /* Corridor sequence when VERIFY began. */
    int32_t start_distance_mm; /* Odometry when the command was sent. */
    int32_t start_heading_deg;
    uint32_t leg_timeout_ms;
} obstacle_avoid_t;

/* Public function declarations. */

/**
 * @brief Plan a bypass from a published side result, best effort.
 *
 * The line is assumed to continue behind the obstacle. PATH_FOUND and
 * BLOCKED results are both used. Each side whose edge was found (with the
 * width available) needs edge + half_width + margin sideways (-right_x on
 * the right); a side without one needs sidestep_max. PATH_FOUND takes the
 * scanner's measured-clearest direction even if its sidestep is longer or
 * its edge is unknown (then at sidestep_max, b_edge_unknown). BLOCKED goes
 * round the found edge needing the smaller sidestep; with no edge found it
 * has nothing to try (NOT_PATH).
 * The sidestep is raised to sidestep_min. A sidestep above sidestep_max or
 * a pass above pass_max is capped there and b_limited set: the car tries
 * anyway, guarded by CRITICAL. The pass distance is centre + axle_to_pivot
 * + depth + rear + margin.
 *
 * @param result Published side result, PATH_FOUND or BLOCKED.
 * @param geometry Geometry published with result.
 * @param p_geometry Non-null car dimensions and limits.
 * @return The plan; check status before using the distances.
 */
obstacle_avoid_plan_t
obstacle_avoid_plan (robot_obstacle_result_t result,
                     obstacle_scan_geometry_t geometry,
                     obstacle_avoid_geometry_t const * p_geometry);

/**
 * @brief Count the motion commands needed to drive a distance.
 * @param distance_mm Distance in millimetres.
 * @return 0 for 0, else the number of parts of at most
 *         ROBOT_MOTION_CMD_DISTANCE_MAX_MM each.
 */
uint8_t obstacle_avoid_split_count (uint32_t distance_mm);

/**
 * @brief Size the next command of a split move.
 * @param remaining_mm Distance still to drive.
 * @return The remaining distance divided evenly over
 *         obstacle_avoid_split_count() parts, rounded up; 0 for 0.
 */
uint16_t obstacle_avoid_next_part (uint32_t remaining_mm);

/**
 * @brief Check a corridor sweep against the path the car will pass along.
 *
 * Each usable non-zero echo at angle 60 + 5 i degrees gives a point at
 * x = r cos(angle), y = axle_to_pivot + r sin(angle) from the axle. Points
 * with y below pass_mm + axle_to_pivot are in range. Let g be x measured
 * toward the gap. A point in range with |x| < half_width + margin is in the
 * path; the shift is the largest g + half_width + margin over those, which
 * puts each one on the far side. A point in range on the gap side outside
 * the path (g >= half_width + margin) limits the shift to
 * g - half_width - margin. The 60-120 degree sweep cannot see beside the
 * front wheels; the first sweep's edges cover that area, this does not.
 *
 * @param p_corridor Non-null corridor sweep, taken with the car stopped.
 * @param p_width Non-null range corrections.
 * @param p_geometry Non-null car dimensions.
 * @param pass_mm Axle travel still to come parallel to the line.
 * @param b_gap_right True when the bypass is on the right.
 * @param p_shift_mm Non-null; set to the shift for SHIFT, else 0.
 * @return CLEAR with nothing in the path; SHIFT with *p_shift_mm; BLOCKED
 *         if any position is invalid, a range cannot be computed, or the
 *         shift would move the car into a gap-side point.
 */
obstacle_avoid_corridor_t
obstacle_avoid_check_corridor (obstacle_scan_corridor_t const * p_corridor,
                               obstacle_width_config_t const * p_width,
                               obstacle_avoid_geometry_t const * p_geometry,
                               uint16_t pass_mm, bool b_gap_right,
                               uint16_t * p_shift_mm);

/**
 * @brief Start the state machine in WAIT_CLEAR with INHIBIT assumed set.
 *
 * The caller sets INHIBIT before the scan task starts, so the car waits for
 * the first confirmed clear path before it follows the line.
 * @param p_ctx Non-null context to initialise.
 * @param p_config Non-null configuration; kept by pointer, must outlive it.
 * @param now_ms Current kernel time in milliseconds.
 */
void obstacle_avoid_init (obstacle_avoid_t * p_ctx,
                          obstacle_avoid_config_t const * p_config,
                          uint32_t now_ms);

/**
 * @brief Advance the bypass by one scan-task loop.
 *
 * Never blocks. Any failed INHIBIT set or clear, failed event clear or send
 * error latches FAULT, as do encoder faults, rejected commands, missing
 * odometry, a late scanner and a second stall, timeout or missed line in
 * one bypass. FAULT keeps INHIBIT set and ignores every later input.
 * @param p_ctx Context from obstacle_avoid_init().
 * @param p_in Non-null inputs for this loop.
 * @return Actions to carry out now, in the order documented above.
 */
obstacle_avoid_action_t
obstacle_avoid_step (obstacle_avoid_t * p_ctx,
                     obstacle_avoid_input_t const * p_in);

/**
 * @brief Name a state for logging.
 * @param state State to name.
 * @return Static string, never NULL; "INVALID" for unknown values.
 */
char const * obstacle_avoid_state_name (obstacle_avoid_state_t state);

#endif /* OBSTACLE_AVOID_H */
