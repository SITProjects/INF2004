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
#include <stdint.h>

/* Public types, constants, and macros. */

typedef enum
{
    ULTRASONIC_PENDING = 0,
    ULTRASONIC_VALID,
    ULTRASONIC_NO_ECHO,
    ULTRASONIC_FAULT
} ultrasonic_result_t;

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
 * Uses the trigger timing and minimum ping interval in obstacle_config.h.
 * @return False while a measurement is pending, unread, or within the
 *         minimum ping interval. True accepts a transaction; if ECHO is
 *         already high, read returns FAULT without sending a trigger.
 */
bool ultrasonic_trigger (void);

/**
 * @brief Poll and consume a measurement.
 *
 * @param echo_us Non-null output. Receives the pulse duration for VALID,
 *        zero for NO_ECHO or FAULT, and remains unchanged for PENDING.
 *
 * Start timeout is measured from the trigger; pulse timeout from the rise.
 * Completed results are consumed once. The scanner applies no-return policy.
 * @return ULTRASONIC_PENDING: no completed result.
 *         ULTRASONIC_VALID: a valid pulse was captured.
 *         ULTRASONIC_NO_ECHO: no rise before the start timeout, or a pulse
 *         reaching the measurable range/storage limit. Late edges are ignored.
 *         ULTRASONIC_FAULT: malformed edges, zero pulse or ECHO already high
 *         when a new trigger is attempted. A pulse range expiry alone is not
 *         a FAULT; hardware no-return behavior still needs scope validation.
 *
 * Only the sensor-owning task may call this function.
 */
ultrasonic_result_t ultrasonic_read (uint16_t * echo_us);

#endif /* DRV_ULTRASONIC_H */
