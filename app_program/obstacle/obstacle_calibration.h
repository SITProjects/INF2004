/**
 * @file obstacle_calibration.h
 * @brief Raw ultrasonic and servo calibration in the sensor-owning task.
 */
#ifndef OBSTACLE_CALIBRATION_H
#define OBSTACLE_CALIBRATION_H

/* Public includes. */

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Initialise the sensor and servo, then run the selected mode forever.
 * @param trig_pin Trigger output GPIO.
 * @param echo_pin Echo input GPIO.
 * @param servo_pin Servo PWM output GPIO.
 * @pre OBSTACLE_CALIBRATION_MODE is 1..4. Call once from the sensor-owning
 *      task on core 0; no scanner or other task may access these drivers.
 * @return Never returns. Initialisation failure logs an error and terminates
 *         the calling task. The implementation is absent in mode 0 builds.
 */
_Noreturn void obstacle_calibration_run (int trig_pin, int echo_pin,
                                         int servo_pin);

#endif /* OBSTACLE_CALIBRATION_H */
