/**
 * @file motion_command.h
 * @brief Pass the line task's requests (IF-04) to the motion module.
 */

#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include "../app.h"
#include "../line_follow/line_track.h"

/* Public types, constants, and macros. */

/* Actuation, speeds and startup delay are in motion_command_config.h. */

/* Public function declarations. */

/**
 * @brief Create the latest-request mutex before tasks start.
 * @return E_OK, E_OBJ if already initialized, or a negative kernel error.
 * @note Called once by motion_init(). Does not access the motors.
 */
ER motion_command_init (void);

/**
 * @brief Prepare the adapter and record its nonblocking startup delay.
 * @pre motion_command_init(), motion_drive_init(), motion_move_init() succeeded.
 * @return E_OK, E_OBJ if not initialized/already started, or a kernel error.
 * @note Called once by motion_start(), before creating tMotion. No task is
 *       created here. tMotion owns all motor/encoder access afterwards.
 */
ER motion_command_start (void);

/**
 * @brief Service the latest line request from the sole motor owner, tMotion.
 * @pre motion_command_start() succeeded. Call once per motion tick before
 *      motion_move_update(); this function never advances that control loop.
 * @param b_control_enabled True permits line requests to drive after the
 *        startup delay. False suspends them for queued commands/inhibit,
 *        while fresh line visibility still feeds stop-at-line movements.
 * @note Only tMotion may call. No waiting or sleeping. A request older than
 *       MOTION_LINE_TIMEOUT_MS stops automatic following.
 *       TRACK follows at MOTION_COMMAND_FOLLOW_SPEED_MM_S on the requested
 *       heading. SEARCH_LEFT/RIGHT pivots once towards the requested
 *       heading; a new target starts a new pivot. STOP brakes.
 *       APP_TEST and disabled actuation only log; they never call motor APIs.
 */
void motion_command_service (bool b_control_enabled);

/**
 * @brief Replace the latest line request.
 * @pre motion_command_init() succeeded; the line task, either core.
 * @param p_request Copied before returning. NULL means a sensor failure
 *        and requests a stop with the line not seen.
 * @return E_OK, E_PAR for an invalid mode (stop request stored), E_TMOUT
 *         if the slot is busy (sample skipped), or another kernel error.
 * @note Never waits for the mutex. On E_TMOUT the caller retries with its
 *       next sample; the target heading is absolute, so a skipped sample
 *       loses nothing.
 */
ER motion_command_submit (line_request_t const * p_request);

#endif /* MOTION_COMMAND_H */
