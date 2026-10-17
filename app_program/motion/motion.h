/**
 * @file motion.h
 * @brief Motion subsystem: two H-bridge DC motors. GitHub: felicia-chuaxy.
 *
 * This header is the motion subsystem's PUBLIC interface: only what other
 * subsystems may use. Everything else stays private in the .c files. Other
 * subsystems include it as "../motion/motion.h".
 *
 * The subsystem has no task yet: motion_start() configures both motors and
 * holds them braked. The open-loop drive API is in motion_drive.h; motor
 * pins and calibration are in motion_config.h.
 */

#ifndef MOTION_H
#define MOTION_H

/* Public includes. */
#include "../app.h"

/* Public types, constants, and macros. */

/** The motion priority band is 14-15 (TEAM.md section 4); no task yet. */
#define MOTION_PRI_DRIVE 14

/* Public function declarations. */

#endif /* MOTION_H */
