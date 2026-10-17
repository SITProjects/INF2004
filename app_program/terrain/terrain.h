/**
 * @file terrain.h
 * @brief Terrain subsystem: GY-511 accelerometer on I2C0. GitHub: ann.
 *
 * This header is the terrain subsystem's PUBLIC interface: only what other
 * subsystems may use. Everything else stays private in the .c files. Other
 * subsystems include it as "../terrain/terrain.h".
 *
 * The readings API is in terrain_monitor.h: terrain_monitor_get_latest(),
 * terrain_monitor_get_calibration() and terrain_monitor_recalibrate() are
 * safe to call from any task. Everything else in that module belongs to
 * the terrain task.
 */

#ifndef TERRAIN_H
#define TERRAIN_H

/* Public includes. */
#include "../app.h"

/* Public types, constants, and macros. */

/** Terrain task priority; the terrain band is 18-19 (TEAM.md section 4). */
#define TERRAIN_PRI_MONITOR 18

/** ID of the terrain task, which owns I2C0. */
extern ID terrain_tid_monitor;

/* Public function declarations. */

#endif /* TERRAIN_H */
