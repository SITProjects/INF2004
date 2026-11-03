/**
 * @file telemetry_queue_port.h
 * @brief Private kernel mutex adapter; keeps kernel headers away from lwIP.
 */
#ifndef TELEMETRY_QUEUE_PORT_H
#define TELEMETRY_QUEUE_PORT_H

/* Public includes. */
#include <stdbool.h>

/* Public function declarations. */
/** @brief Create one priority-inheritance mutex; zero or negative error. */
int telemetry_queue_port_init (void);
/** @brief Attempt the mutex with TMO_POL; false before init or if busy. */
bool telemetry_queue_port_lock (void);
/** @brief Release after a successful lock; task context only. */
void telemetry_queue_port_unlock (void);

#endif /* TELEMETRY_QUEUE_PORT_H */
