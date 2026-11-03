/**
 * @file motion.h
 * @brief Motion subsystem: two H-bridge DC motors. GitHub: felicia-chuaxy.
 *
 * This header is the motion subsystem's PUBLIC interface: only what other
 * subsystems may use. Everything else stays private in the .c files. Other
 * subsystems include it as "../motion/motion.h".
 *
 * The motion task (tMotion) owns the motors (PWM slices 4 and 5) and the
 * encoders (PIO0). It wakes every MOTION_UPDATE_PERIOD_MS (CYC_MOTION), or
 * at once on MOTION_FLG_INHIBIT, runs the commands queued in
 * motion_mbf_cmd, and reports through motion_flg_event,
 * motion_get_status() and motion_get_odometry(). Other tasks never call
 * motion_drive_ or motion_move_ functions.
 *
 * To command the car (tMission): pack a word with the robot_motion_cmd_
 * helpers in robot_types.h and send it with
 * tk_snd_mbf(motion_mbf_cmd, &cmd, sizeof(cmd), tmout). A new command
 * replaces the one in progress, which then ends without MOVE_DONE.
 *
 * To stop the car (tSafety): tk_set_flg(motion_flg_ctrl,
 * MOTION_FLG_INHIBIT). tMotion wakes and brakes at once (NFR6), discards
 * every queued and new command, and reports ROBOT_MOTION_MODE_INHIBITED
 * until tk_clr_flg(motion_flg_ctrl, ~MOTION_FLG_INHIBIT).
 */

#ifndef MOTION_H
#define MOTION_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include "../app.h"
#include "../robot_types.h"

/* Public types, constants, and macros. */

/** Priority of tMotion, in the motion band 14-15 (TEAM.md section 4). */
#define MOTION_PRI_DRIVE 14

/** motion_flg_ctrl (FLG_MOTION): set by the CYC_MOTION handler. */
#define MOTION_FLG_TICK 0x01u
/** motion_flg_ctrl (FLG_MOTION): MOTOR_INHIBIT; set and cleared by tSafety. */
#define MOTION_FLG_INHIBIT 0x02u

/*
 * motion_flg_event bits. tMotion sets them; the waiting task clears the
 * ones it has handled with tk_clr_flg(motion_flg_event, ~bits).
 * TODO(step 9, tMission): move MOVE_DONE and STALL into FLG_MISSION, and
 * STALL / ENC_FAULT into the shared fault mask, once tMission owns them.
 */
/** A move or turn reached its target or the line; see the done reason. */
#define MOTION_EVT_MOVE_DONE 0x01u
/** A wheel stalled (FR12); braked, mode STALLED until STOP or INHIBIT. */
#define MOTION_EVT_STALL 0x02u
/** A move or turn timed out (encoder fault trigger); mode TIMED_OUT. */
#define MOTION_EVT_ENC_FAULT 0x04u
/** A command was invalid or refused (e.g. while STALLED); nothing ran. */
#define MOTION_EVT_REJECTED 0x08u
/** Following ended because line readings stopped arriving. */
#define MOTION_EVT_FOLLOW_ENDED 0x10u

/**
 * Odometry as of tMotion's last update (IF-05, motion -> terrain). Units as
 * motion_move_get_odometry(); + is forward and left.
 */
typedef struct
{
    uint32_t updates;         /**< tMotion updates published since start-up;
                                   stops rising when tMotion stops. */
    int32_t distance_mm;      /**< Mean of both wheels' travel. */
    int32_t heading_deg;      /**< Total turn, not wrapped. */
    int16_t speed_mm_s;       /**< Mean of the two wheel speeds. */
    int16_t left_speed_mm_s;  /**< Left wheel speed. */
    int16_t right_speed_mm_s; /**< Right wheel speed. */
} motion_shared_odometry_t;

/** Mode and done reason as of tMotion's last update. */
typedef struct
{
    robot_motion_mode_t mode; /**< Current mode. */
    robot_motion_done_t done; /**< Why the last move or turn ended. */
} motion_task_status_t;

/**
 * tMotion loop timing (NFR1, NFR6), from the 1 MHz TIMER, since tMotion
 * started or was last released by motion_task_hold(false).
 */
typedef struct
{
    uint32_t updates;          /**< Update wake-ups (ticks) counted. */
    uint32_t min_period_us;    /**< Shortest time between two updates. */
    uint32_t max_period_us;    /**< Longest time between two updates. */
    uint32_t max_busy_us;      /**< Longest update, wake-up to loop end. */
    uint32_t inhibit_brake_us; /**< TIMER when INHIBIT last braked. */
} motion_task_timing_t;

/** Event flag FLG_MOTION: MOTION_FLG_TICK and MOTION_FLG_INHIBIT. */
extern ID motion_flg_ctrl;

/** Message buffer MBF_MOTION_CMD: one uint32_t command word per message. */
extern ID motion_mbf_cmd;

/** Event flag of motion events (MOTION_EVT_ bits); several may wait. */
extern ID motion_flg_event;

/* Public function declarations. */

/**
 * @brief Read tMotion's mode and done reason.
 * @return Snapshot as of tMotion's last update; ROBOT_MOTION_MODE_STOPPED
 * before the first.
 * Any task, either core: the snapshot is one 32-bit word.
 */
motion_task_status_t motion_get_status (void);

/**
 * @brief Copy tMotion's latest odometry (IF-05).
 * TODO(step 8): tMotion also copies these into telemetry_t under
 * MTX_TELEM once it exists.
 * @param p_odometry Caller-owned snapshot to fill.
 * @return true with a consistent snapshot (updates 0 before the first
 * update); false for NULL, or if tMotion was writing on every try (only
 * possible from a task above tMotion on the same core): try again later.
 * Any task, either core; never waits or locks.
 */
bool motion_get_odometry (motion_shared_odometry_t * p_odometry);

/**
 * @brief Pass in the latest line reading (interim line input).
 * TODO(step 9, line_follow): replace with tMotion reading line_state_t
 * under MTX_LINE once line_follow publishes it; this then goes.
 * @param error Line position, -1000 to 1000 (clamped); + means the line
 * is left of the car's centre. When lost, the last sign at full scale.
 * @param b_seen False only when both sensors read white.
 * Call from ONE task (the line owner), once per new reading; tMotion uses
 * each reading once, on its next update. Either core: one 32-bit store.
 */
void motion_set_line_input (int16_t error, bool b_seen);

#if APP_TEST
/**
 * @brief Hand the motion module to the test task, or back to tMotion.
 * @param b_hold True: tMotion brakes and stops touching the hardware or
 * the motion module, so the caller may call motion_drive_ and
 * motion_move_ functions itself. False: tMotion takes them back and
 * restarts its timing statistics.
 * @return E_OK once tMotion has acknowledged, or E_TMOUT.
 * Test task only. Blocks for up to a few update periods.
 */
ER motion_task_hold (bool b_hold);

/**
 * @brief Read tMotion's loop timing.
 * @param p_timing Caller-owned snapshot to fill.
 * Call only while held (motion_task_hold(true)), so the values are whole.
 */
void motion_task_get_timing (motion_task_timing_t * p_timing);
#endif

#endif /* MOTION_H */
