/**
 * @file drv_ir_sensor.h
 * @brief Raw analogue readings from two reflective IR line sensors.
 *
 * Reads through the kernel's A/D converter device driver ("adca"). The
 * kernel configures GP26 and GP27 as analogue inputs at boot
 * (DEVCONF_ADC_PIN_INIT_0/1 in device/adc/sysdepend/rp2040/
 * adc_cnf_sysdep.h). Only one task may use the ADC (TEAM.md section 3).
 */

#ifndef DRV_IR_SENSOR_H
#define DRV_IR_SENSOR_H

/* Public includes. */
#include <stdbool.h>
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
 * @brief Open the kernel's ADC device for the sensor inputs.
 * @return True on success; false if the device cannot be opened.
 * Call once, from the task that will read the sensors.
 */
bool ir_sensor_init (void);

/**
 * @brief Read left, then right, using blocking ADC conversions.
 * @note Discard the first conversion after each channel selection and keep
 *       the second unchanged. Four conversions produce two unfiltered
 *       samples.
 * @pre ir_sensor_init() succeeded; call from the same task.
 * @param p_reading Receives raw ADC counts, not simultaneous samples or
 *        surface classifications. Unspecified after a failure.
 * @return True on success; false if a conversion failed.
 */
bool ir_sensor_read (ir_sensor_reading_t * p_reading);

#endif /* DRV_IR_SENSOR_H */
