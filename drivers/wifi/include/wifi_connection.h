/**
 * @file wifi_connection.h
 * @brief TODO: Describe the module responsibility.
 *
 */

#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include <stdbool.h>
/**
 * @brief Initialises the Wi-Fi in the Pico W
 * @return true if connection suceeds, false otherwise
 */
bool wifi_init (const char * ssid, const char * password);

/**
 * @brief Monitors the Wi-Fi Connection, and attempts to reconnect
 * until connection is restored.
 */
void wifi_monitor (void);
/**
 * @brief Prints IP Address, Subnet Mask and Default Gateway
 */
void wifi_print_network_info (void);

#endif
