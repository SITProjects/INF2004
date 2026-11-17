/**
 * @file barcode.h
 * @brief Barcode subsystem: reads Code 39 with a third IR sensor.
 *
 * This header is the subsystem's PUBLIC interface: only what other
 * subsystems may use. Everything else stays private in the .c files.
 * Other subsystems include it as "../barcode/barcode.h".
 */

#ifndef BARCODE_H
#define BARCODE_H

/* Public includes. */
#include "../app.h"

/* Public types, constants, and macros. */

/** tBarcode priority, reserved next to the line task (TEAM.md section 4). */
#define BARCODE_PRI_READ 8

/** ID of tBarcode, which owns GP28 (core 0: the GPIO interrupt's core). */
extern ID barcode_tid_read;

/* Public function declarations. */

#endif /* BARCODE_H */
