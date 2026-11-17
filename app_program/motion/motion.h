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
 * With the line adapter enabled, FOLLOW resumes automatic line decisions;
 * STOP/MOVE/TURN suspend them until a new FOLLOW. INHIBIT also suspends
 * them, so clearing it alone cannot replay a cached line command.
 * Queued MOVE/TURN with stop-at-line acquires any fresh, confirmed visible
 * line, including one already under the sensors when the command starts.
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

/** Priority of tMotion: design freeze 3 + 3 (TEAM.md section 4). */
#define MOTION_PRI_DRIVE 6

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
/** Every MOTION_EVT_ bit, for clearing them all before a new command. */
#define MOTION_EVT_ALL 0x1Fu

/**
 * Odometry as of tMotion's last update (IF-05, motion -> terrain). Units as
 * motion_move_get_odometry(); + is forward and left.
 */
typedef robot_odometry_t motion_shared_odometry_t;

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
 * tMotion publishes the status before it sets any MOTION_EVT_ bit, so a
 * task that reads an event and then calls this sees the done reason of
 * the command that raised it.
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
 * @brief Whether tMotion follows the line task's requests.
 * @return True after start-up and after a FOLLOW command; false after any
 * other queued command or INHIBIT until the next FOLLOW.
 * Any task, either core: one byte written by tMotion only. The line task
 * uses it to restart tracking on the heading a bypass leaves it on.
 */
bool motion_line_control_active (void);

#if APP_TEST
/**
 * @brief Test line input (MOT-U36): pass whether the line is seen.
 * @param b_seen True for a line under either sensor.
 * Test task only, once per update; tMotion uses each reading once.
 */
void motion_test_set_line (bool b_seen);

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
