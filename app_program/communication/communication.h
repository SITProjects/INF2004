/**
 * @file communication.h
 * @brief Communication subsystem: Wi-Fi status. GitHub: Jerome-4469.
 *
 * This header is the communication subsystem's PUBLIC interface: only what
 * other subsystems may use. Everything else stays private in the .c files.
 * Other subsystems include it as "../communication/communication.h".
 *
 * connectivity_is_connected() in connectivity.h is safe from any task.
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/* Public includes. */
#include "../app.h"

/* Public types, constants, and macros. */

/** Wi-Fi task priority; the communication band is 20-21 (TEAM.md). */
#define COMMUNICATION_PRI_WIFI 20

/** ID of the Wi-Fi status task. */
extern ID communication_tid_wifi;

/* Public function declarations. */

#endif /* COMMUNICATION_H */
