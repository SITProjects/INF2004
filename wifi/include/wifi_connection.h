/**
 * @file wifi_connection.h
 * @brief TODO: Describe the module responsibility.
 *
 */

#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include <stdbool.h>

bool wifi_connect(const char *ssid, const char *password);
void wifi_print_network_info(void);
void wifi_deinit(void);

#endif
