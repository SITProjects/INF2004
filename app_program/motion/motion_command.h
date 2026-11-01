/**
 * @file motion_command.h
 * @brief Task-safe handoff from line decisions to the motor owner task.
 */

#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include "../app.h"
#include "../line_follow/line_command.h"

/* Public types, constants, and macros. */

/** Motor movement is opt-in; APP_TEST builds always keep motors braked. */
#ifndef MOTION_COMMAND_ACTUATION_ENABLED
#define MOTION_COMMAND_ACTUATION_ENABLED 0
#endif

/* Public function declarations. */

/**
 * @brief Create the latest-command mutex and event flag.
 * @return E_OK, E_OBJ if already initialized, or a negative kernel error.
 * @note Called once by motion_init(), before tasks start. No motor access.
 */
ER motion_command_init (void);

/**
 * @brief Start the task that owns motor calls after startup initialization.
 * @pre motion_command_init() and motion_drive_init() succeeded.
 * @return E_OK, E_OBJ if not initialized or already started, or kernel error.
 * @note Called once by motion_start(). Task uses the reserved motion priority.
 */
ER motion_command_start (void);

/**
 * @brief Replace the latest line command and notify the motor task.
 * @pre motion_command_init() succeeded. Call from task context on either core.
 * @param command Decision; SEARCH, JUNCTION and STOP request braking.
 * @return E_OK on submission, E_PAR for an invalid command (STOP submitted),
 *         or negative kernel error. Failed submission guarantees no delivery.
 * @note Nonblocking submission; retries can be made on the next sensor sample.
 *       A mutex protects the copied command and monotonic timestamp. No caller
 *       storage is retained and older commands do not accumulate in a queue.
 *       The motor task brakes when the last request reaches 250 ms of age,
 *       subject to kernel scheduling, and on receive/driver errors. Other
 *       tasks must not call motion_drive_* once the motor task has started.
 */
ER motion_command_submit (line_command_t command);

#endif /* MOTION_COMMAND_H */
