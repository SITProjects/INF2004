/**
 * @file connectivity.h
 * @brief Wi-Fi connection monitoring.
 *
 * The radio, the network stack and the join and retry logic belong to the
 * kernel's Wi-Fi service task (cyw43_utk_task). Application tasks must not
 * call cyw43_ or lwIP functions (TEAM.md section 3). This module only
 * reads the service's status and reports changes. The network name and
 * password come from config/wifi_secrets.h; joining needs a NET=lwip build,
 * which is the default.
 */
#ifndef CONNECTIVITY_H
#define CONNECTIVITY_H

/* Public includes. */
#include <stdbool.h>

/* Public function declarations. */

/**
 * @brief Start monitoring, and report whether this build joins a network.
 * Call once, from the task that calls connectivity_update().
 */
void connectivity_init (void);

/**
 * @brief Report link changes: the IP address on connecting, and a notice
 *        when the link drops. Never waits.
 * @note Call regularly from one task. Does nothing before
 *       connectivity_init() or on a build that does not join a network.
 */
void connectivity_update (void);

/**
 * @brief Check the current link status.
 * @return true if the station has joined and has an IP address. Safe to
 *         call from any task.
 */
bool connectivity_is_connected (void);

#endif /* CONNECTIVITY_H */
