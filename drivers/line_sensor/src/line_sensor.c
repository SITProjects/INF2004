/**
 * @file line_sensor.c
 * @brief Sequential ADC readings from two reflective IR line sensors.
 *
 */

#include "line_sensor.h"
#include "hardware/adc.h"

/* Private types, constants, and macros. */
#define LINE_SENSOR_LEFT_PIN 26u
#define LINE_SENSOR_LEFT_CHANNEL 0u
#define LINE_SENSOR_RIGHT_PIN 27u
#define LINE_SENSOR_RIGHT_CHANNEL 1u

/* Public function definitions. */

void
line_sensor_init (void)
{
    adc_init();
    adc_gpio_init(LINE_SENSOR_LEFT_PIN);
    adc_gpio_init(LINE_SENSOR_RIGHT_PIN);
    adc_select_input(LINE_SENSOR_LEFT_CHANNEL);
}

line_sensor_reading_t
line_sensor_read (void)
{
    adc_select_input(LINE_SENSOR_LEFT_CHANNEL);
    uint16_t const left_raw = adc_read();
    adc_select_input(LINE_SENSOR_RIGHT_CHANNEL);
    uint16_t const right_raw = adc_read();
    line_sensor_reading_t const reading = {
        .left_raw = left_raw,
        .right_raw = right_raw,
    };
    return reading;
}
