/**
 * @file main.c
 * @brief Vehicle controller application entry point.
 *
 */

#include "main.h"
#include "obstacle.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "connectivity.h"
#include <stdio.h>

/* Private types, constants, and macros. */

#define MAIN_LOOP_SLEEP_MS 1u
#define CONNECTIVITY_UPDATE_MS 100u

/* Static data. */

/* Preserve the one-second wait until travel and settling are measured. */
static const obstacle_servo_timing_t obstacle_servo_timing = {
    .travel_ms_per_60_degrees = 0u, .settle_margin_ms = 1000u};

/* Private function declarations. */
#include "credentials.h"

static int init_modules (void);
static void log_scan_result (const obstacle_scan_result_t * scan);

/* Public function definitions. */

int
main (void)
{
    stdio_init_all();
    if (init_modules())
    {
        printf("Failed to initialize modules\n");
        return 1;
    }

    uint32_t last_scan_sequence = 0u;
    absolute_time_t connectivity_deadline = make_timeout_time_ms(0u);
    while (true)
    {
        if (time_reached(connectivity_deadline))
        {
            connectivity_update();
            connectivity_deadline =
                make_timeout_time_ms(CONNECTIVITY_UPDATE_MS);
        }

        obstacle_scan_result_t scan = obstacle_scan();
        bool new_result = scan.sequence != last_scan_sequence;
        last_scan_sequence = scan.sequence;
        if (new_result)
        {
            log_scan_result(&scan);
        }
        switch (scan.status)
        {
            case OBSTACLE_CLEAR:
                /* TODO: Allow forward motion. */
                break;
            case OBSTACLE_PATH_FOUND:
                if (!new_result)
                {
                    break;
                }
                /* TODO: Start a turn toward scan.path_found_angle once. */
                break;
            case OBSTACLE_SCANNING:
            case OBSTACLE_BLOCKED:
            case OBSTACLE_UNKNOWN:
                /* TODO: Keep motion stopped until a direction is available. */
                break;
        }
        /* Echo edges remain interrupt-driven while the main loop sleeps. */
        sleep_ms(MAIN_LOOP_SLEEP_MS);
    }
}

/* Private function definitions. */

static void
log_scan_result (const obstacle_scan_result_t * scan)
{
    switch (scan->status)
    {
        case OBSTACLE_PATH_FOUND:
            printf("Path found at angle %d degrees, ", scan->path_found_angle);
            break;
        case OBSTACLE_BLOCKED:
            printf("Obstacle blocked, ");
            break;
        case OBSTACLE_UNKNOWN:
            /* Include partial spans from a side scan with a missing echo. */
            if (scan->contributing_sides == 0u)
            {
                return;
            }
            printf("Obstacle unknown, ");
            break;
        default:
            return;
    }

    if (scan->sampled_span_cm < 0.0f)
    {
        printf("sampled span unavailable\n");
    }
    else
    {
        printf("sampled span %.2f cm (%u contributing sides)\n",
               scan->sampled_span_cm, (unsigned int)scan->contributing_sides);
    }
}

static int
init_modules (void)
{
    if (obstacle_init(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN, OBSTACLE_SERVO_PIN,
                      &obstacle_servo_timing))
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
