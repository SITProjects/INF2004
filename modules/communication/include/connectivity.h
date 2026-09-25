/**
 * @file connectivity.h
 * @brief Wi-Fi connection monitoring and retry policy.
 */
#ifndef CONNECTIVITY_H
#define CONNECTIVITY_H

/* Public includes. */
#include <stdbool.h>

/* Public function declarations. */
/**
 * @brief Initialize connectivity once at startup and start connecting.
 * @param ssid Non-NULL, null-terminated name, borrowed for the module lifetime.
 * @param password Non-NULL, null-terminated password, borrowed for module life.
 * @return true if initialized (connection may be pending), false on invalid
 * arguments, repeated initialization, or hardware initialization failure.
 * @note Credentials must remain valid and unchanged for the module lifetime.
 * All connectivity APIs must be called from the main loop context.
 */
int connectivity_init (const char * ssid, const char * password);

/**
 * @brief Advance connection state without waiting for association or DHCP.
 * @note Call regularly. Attempts time out after 10 seconds; retries start
 * after a 5-second delay. A late connection is still accepted.
 * Does nothing before successful initialization.
 */
void connectivity_update (void);

/**
 * @brief Check the current driver link status.
 * @return true if initialized and connected with an IP address, false otherwise.
 */
bool connectivity_is_connected (void);

#endif /* CONNECTIVITY_H */
