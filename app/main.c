/**
 * @file main.c
 * @brief Application entry point placeholder.
 *
 */

#include "main.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "motion.h"
#include "obstacle.h"

int init_modules (void);

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
        if (obstacle_update_check())
        {
            if (is_obstacle_detected())
            {
                printf("OBSTACLE TOO CLOSE!\n");
            }
        }

        tight_loop_contents();
    }
}

int
init_modules (void)
{
    if (obstacle_init(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN))
    {
        return 1;
    }
    irq_set_enabled(IO_IRQ_BANK0, true);
    return 0;
}
