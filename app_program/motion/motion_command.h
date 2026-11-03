/**
 * @file motion_command.h
 * @brief Pass real IR readings to the existing motion follow controller.
 */

#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include "../app.h"
#include "../line_follow/line_command.h"

/* Public types, constants, and macros. */

/* Actuation, speeds and startup delay are in motion_command_config.h. */

/* Public function declarations. */

/**
 * @brief Create the latest-reading mutex before tasks start.
 * @return E_OK, E_OBJ if already initialized, or a negative kernel error.
 * @note Called once by motion_init(). Does not access the motors.
 */
ER motion_command_init (void);

/**
 * @brief Start the periodic task that owns the motion controller.
 * @pre motion_command_init(), motion_drive_init(), motion_move_init() succeeded.
 * @return E_OK, E_OBJ if not initialized/already started, or a kernel error.
 * @note Called once by motion_start(). Motor/encoder ownership transfers to
 *       this task in enabled normal builds. In dry runs and APP_TEST it never
 *       calls motion APIs, allowing existing motion tests exclusive access.
 *       Enabled operation starts after the configured preparation delay and
 *       runs continuously. IR_MOTOR_TEST buffers 32 snapshots and prints
 *       them once after the first stop; it never prints the trace in motion.
 */
ER motion_command_start (void);

/**
 * @brief Replace the latest IR reading and navigation decision.
 * @pre motion_command_init() succeeded; task context on either core.
 * @param command STOP brakes. SEARCH curves slowly toward the last known
 *        side until new edge evidence arrives; there is no search-fail timer.
 *        PROBE_LEFT/RIGHT keeps a small remembered bias between edge detections.
 *        ALIGN_LEFT/RIGHT uses the detected edge correction at SEARCH speed,
 *        keeping reacquisition slow until the line task ends alignment.
 *        Through brief white gaps it uses the remembered gentle probe bias.
 *        FORWARD/JUNCTION crosses straight; PD may briefly counter-steer.
 *        TURN_LEFT/RIGHT requests one encoder-controlled 90-degree turn on
 *        confirmed both-black reacquisition during SEARCH. Fresh valid IR
 *        samples are required throughout; side decisions cannot interrupt
 *        the turn. STOP still brakes. After completion, normal decisions
 *        resume immediately. A held turn request resumes slow SEARCH instead
 *        of repeating the pivot; a new non-turn sample rearms the turn.
 * @param p_reading Raw ADC pair, 0..4095 per channel, copied before returning.
 *        NULL means sensor failure and requests a stop without line feedback.
 * @return E_OK, E_PAR for invalid counts/command (stop request stored),
 *         E_TMOUT if the slot is busy (sample skipped), or another kernel
 *         error. Failed submission guarantees no delivery.
 * @note Submission never waits for the mutex. On E_TMOUT the caller may
 *       skip logging and retry with its next sample. Receiver contention
 *       retains a fresh cached request without refreshing its timestamp or
 *       feeding it again to the controller. Shared data is copied under the
 *       mutex; newer readings replace older ones. The owner derives a
 *       bounded error from the sensor detecting the line: left-only is
 *       positive, right-only is negative. Each uses its own detection and
 *       full-black calibration. First detection requests gentle correction;
 *       stronger coverage increases it up to the configured maximum. Probing
 *       uses the explicit remembered-side command, not a low left ADC alone.
 *       The motor owner remembers that side for basic low-speed SEARCH, using
 *       the existing PD controller. Speed changes restart following once per
 *       fast/slow transition; all input still requires fresh ADC pairs.
 *       STOP, FORWARD/JUNCTION and pending full-black crossings feed zero.
 *       PD damping may briefly counter-steer as any correction changes.
 *       Only fresh readings feed the controller. Stale input stops following
 *       using main's MOTION_LINE_TIMEOUT_MS, subject to task scheduling.
 *       Motor speed, PD tuning and cadence reuse motion_config.h. Faults
 *       latch until restart; this adapter never automatically clears a stall.
 */
ER motion_command_submit (line_command_t command,
                          ir_sensor_reading_t const * p_reading);

#endif /* MOTION_COMMAND_H */
