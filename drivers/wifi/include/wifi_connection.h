/**
 * @file wifi_connection.h
 * @brief Low-level Pico W Wi-Fi operations.
 */
#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

/* Public includes. */
#include <stdbool.h>

/* Public function declarations. */
/**
 * @brief Initialize Wi-Fi hardware and enable station mode once at startup.
 * @return true on success, false on SDK initialization failure.
 */
bool wifi_init (void);

/**
 * @brief Start a WPA2 connection without waiting for association or DHCP.
 * @param ssid Non-NULL, null-terminated name, borrowed during this call.
 * @param password Non-NULL, null-terminated password, borrowed during this call.
 * @pre wifi_init() succeeded; call from the main loop.
 * @return true if started, false on invalid arguments or SDK error.
 */
bool wifi_connect_start (const char * ssid, const char * password);

/**
 * @brief Check whether the station link is up and has an IP address.
 * @pre wifi_init() succeeded.
 * @return true when connected, false otherwise.
 */
bool wifi_is_connected (void);

/**
 * @brief Print the IP address, subnet mask, and gateway to standard output.
 * @pre wifi_init() succeeded; call from the main loop.
 */
void wifi_print_network_info (void);

#endif /* WIFI_CONNECTION_H */
