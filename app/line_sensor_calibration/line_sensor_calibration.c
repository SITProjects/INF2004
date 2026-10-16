/**
 * @file line_sensor_calibration.c
 * @brief Standalone test application for dual IR line sensing and
 *        line-following decisions.
 * @details Reads raw ADC values from the left and right IR sensors,
 *          passes them to the line-following logic, and prints the
 *          resulting navigation command over USB serial. Decisions run at
 *          a nominal 20 ms interval; serial snapshots print every 200 ms.
 *          This application does not initialize or drive motors.
 */

#include "line_sensor_calibration.h"

#include <stdio.h>

#include "line_follow.h"
#include "ir_sensor.h"
#include "pico/stdlib.h"

/* Private types, constants, and macros. */

#define USB_CONNECT_DELAY_MS 2000u
#define SAMPLE_INTERVAL_MS 20u
#define SERIAL_INTERVAL_MS 200u

/* Public function definitions. */

int
main (void)
{
    stdio_init_all();

    ir_sensor_init();
    line_follow_init();

    sleep_ms(USB_CONNECT_DELAY_MS);

    printf("IR line-following dry run started. Motors disabled.\n");

    absolute_time_t next_serial_time = get_absolute_time();

    for (;;)
    {
        absolute_time_t const next_sample_time =
            make_timeout_time_ms(SAMPLE_INTERVAL_MS);
        ir_sensor_reading_t const reading = ir_sensor_read();

        line_command_t const command = line_follow_update(reading);

        if (time_reached(next_serial_time))
        {
            printf("Left AO: %u | Right AO: %u | Command: %s\n",
                   (unsigned int)reading.left_raw,
                   (unsigned int)reading.right_raw,
                   line_follow_command_name(command));
            next_serial_time = make_timeout_time_ms(SERIAL_INTERVAL_MS);
        }

        /* Serial output can overrun; never replay missed sensor samples. */
        sleep_until(next_sample_time);
    }

    return 0;
}
