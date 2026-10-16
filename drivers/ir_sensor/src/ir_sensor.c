/**
 * @file ir_sensor.c
 * @brief Sequential ADC readings from two reflective IR line sensors.
 *
 */

#include "ir_sensor.h"
#include "hardware/adc.h"

/* Private types, constants, and macros. */
#define IR_SENSOR_LEFT_PIN 26u
#define IR_SENSOR_LEFT_CHANNEL 0u
#define IR_SENSOR_RIGHT_PIN 27u
#define IR_SENSOR_RIGHT_CHANNEL 1u

/* Public function definitions. */

void
ir_sensor_init (void)
{
    adc_init();
    adc_gpio_init(IR_SENSOR_LEFT_PIN);
    adc_gpio_init(IR_SENSOR_RIGHT_PIN);
    adc_select_input(IR_SENSOR_LEFT_CHANNEL);
}

ir_sensor_reading_t
ir_sensor_read (void)
{
    adc_select_input(IR_SENSOR_LEFT_CHANNEL);
    /* Discard the first conversion after switching ADC channels. */
    (void)adc_read();
    uint16_t const left_raw = adc_read();
    adc_select_input(IR_SENSOR_RIGHT_CHANNEL);
    (void)adc_read();
    uint16_t const right_raw = adc_read();
    ir_sensor_reading_t const reading = {
        .left_raw = left_raw,
        .right_raw = right_raw,
    };
    return reading;
}
