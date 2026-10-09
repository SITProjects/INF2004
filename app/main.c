/**
 * @file main.c
 * @brief Vehicle controller application entry point.
 *
 */

#include "main.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "obstacle.h"
#include "hardware/irq.h"
#include "connectivity.h"
#include "credentials.h"

static int init_modules (void);

int
main (void)
{
    stdio_init_all();
    if (init_modules())
    {
        printf("Failed to initialize modules\n");
        return 1;
    }

    while (true)
    {
        connectivity_update();

        obstacle_scan_result_t scan = obstacle_scan();
        switch (scan.status)
        {
            case OBSTACLE_CLEAR:
                /* TODO: Allow forward motion. */
                break;
            case OBSTACLE_PATH_FOUND:
                /* TODO: Turn toward scan.angle before moving forward. */
                break;
            case OBSTACLE_SCANNING:
            case OBSTACLE_BLOCKED:
            case OBSTACLE_UNKNOWN:
                /* TODO: Keep motion stopped until a direction is available. */
                break;
        }
        tight_loop_contents();
    }
}

static int
init_modules (void)
{
    if (obstacle_init(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN, OBSTACLE_SERVO_PIN))
    {
        printf("Failed to initialize obstacle module\n");
        return 1;
    }
    if (connectivity_init(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("Failed to initialize connectivity module\n");
        return 1;
    }
    irq_set_enabled(IO_IRQ_BANK0, true);
    return 0;
}
