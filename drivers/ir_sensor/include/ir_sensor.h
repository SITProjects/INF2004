/**
 * @file ir_sensor.h
 * @brief Raw analogue readings from two reflective IR line sensors.
 *
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

/* Public includes. */
#include <stdint.h>

/* Public types, constants, and macros. */
/** ADC counts from sequential left and right conversions, returned by value. */
typedef struct
{
    uint16_t left_raw;  /**< Left sensor (GP26), 0 to 4095 ADC counts. */
    uint16_t right_raw; /**< Right sensor (GP27), 0 to 4095 ADC counts. */
} ir_sensor_reading_t;

/* Public function declarations. */

/**
 * @brief Initialize the ADC and both sensor inputs, with digital pulls off.
 * @note Requires exclusive access to the shared ADC during initialization
 *       and reads. Concurrent users must serialize the whole operation.
 */
void ir_sensor_init (void);

/**
 * @brief Read left, then right, using blocking ADC conversions.
 * @note Discard the first conversion after each channel selection and return
 *       the second unchanged. Four conversions produce two unfiltered samples.
 * @pre Call ir_sensor_init() first; retain exclusive ADC access.
 * @return Raw ADC counts, not simultaneous samples or surface classifications.
 */
ir_sensor_reading_t ir_sensor_read (void);

#endif /* IR_SENSOR_H */
