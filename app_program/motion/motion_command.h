/**
 * @file motion_command.h
 * @brief Pass real IR readings to the existing motion follow controller.
 */

#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include "../app.h"
#include "../line_follow/line_command.h"

/* Public types, constants, and macros. */

/** Opt-in movement. Disabled or APP_TEST builds only log adapter inputs. */
#ifndef MOTION_COMMAND_ACTUATION_ENABLED
#define MOTION_COMMAND_ACTUATION_ENABLED 0
#endif

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
 */
ER motion_command_start (void);

/**
 * @brief Replace the latest IR reading and navigation decision.
 * @pre motion_command_init() succeeded; task context on either core.
 * @param command Existing edge decision. SEARCH, STOP and JUNCTION brake.
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
 *       signed reflectance error from the existing per-sensor edge bands,
 *       saturating toward full BLACK; it is not a calibrated distance.
 *       Only fresh readings feed the controller. Stale input stops following
 *       using main's MOTION_LINE_TIMEOUT_MS, subject to task scheduling.
 *       Motor speed, PD tuning and cadence reuse motion_config.h. Faults
 *       latch until restart; this adapter never automatically clears a stall.
 */
ER motion_command_submit (line_command_t command,
                          ir_sensor_reading_t const * p_reading);

#endif /* MOTION_COMMAND_H */
