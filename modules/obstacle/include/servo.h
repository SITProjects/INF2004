/**
 * @file servo.h
 * @brief Servo position control.
 */
#ifndef SERVO_H
#define SERVO_H

/**
 * @brief Configure 50 Hz PWM and centre the servo at 90 degrees.
 * @return 0 on success, 1 for an invalid pin or unsupported system clock.
 * The servo owns its PWM slice; do not reconfigure that slice elsewhere.
 */
int servo_init (int servo_pin);

/** @brief Set the position, clamping angles to the range 0 through 180. */
void servo_set_angle (int angle);

#endif /* SERVO_H */
