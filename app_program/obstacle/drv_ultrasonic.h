/**
 * @file drv_ultrasonic.h
 * @brief Single-shot ultrasonic measurements.
 *
 * The echo pin's edge interrupt is delivered to RP2040 core 0, so
 * ultrasonic_init() must run on core 0: in a task created with TP_PRC1 on
 * the dual-core build. The driver owns the GPIO bank interrupt
 * (IO_IRQ_BANK0); no other code may enable GPIO interrupts. Only one task
 * may use the sensor.
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

/* Public includes. */
#include <stdbool.h>

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Configure the pins and the echo interrupt. Initialise once before
 *        use.
 * @param trig_pin GPIO driving the sensor's trigger input.
 * @param echo_pin GPIO reading the sensor's echo output.
 * @return 0 on success; 1 for invalid pins, when not called on core 0, or
 *         when the interrupt handler cannot be registered.
 */
int ultrasonic_init (int trig_pin, int echo_pin);

/**
 * @brief Start a measurement on the configured trigger pin.
 * @return False while a measurement is pending, unread, or within the
 *         minimum ping interval.
 */
bool ultrasonic_trigger (void);

/**
 * @brief Consume a completed measurement, expiring missing echoes after
 *        23 ms.
 * @param distance_cm Non-null; receives the distance in cm, or -1 for an
 *        invalid echo. Unchanged when the function returns false.
 * @return True when a result was consumed.
 */
bool ultrasonic_read (float * distance_cm);

#endif /* DRV_ULTRASONIC_H */
