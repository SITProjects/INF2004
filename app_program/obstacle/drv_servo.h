/**
 * @file drv_servo.h
 * @brief Servo position control.
 */
#ifndef DRV_SERVO_H
#define DRV_SERVO_H

/* Public includes. */

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Configure 50 Hz PWM and centre the servo at 90 degrees.
 * @param servo_pin GPIO driving the servo signal.
 * @return 0 on success, 1 for an invalid pin or unsupported system clock.
 * The servo owns its PWM slice; do not reconfigure that slice elsewhere.
 */
int servo_init (int servo_pin);

/** @brief Set the position, clamping angles to the range 0 through 180. */
void servo_set_angle (int angle);

#endif /* DRV_SERVO_H */
