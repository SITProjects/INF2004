/**
 * @file main.c
 * @brief Application entry point placeholder.
 *
 */

#include "main.h"
#include <stdio.h>
#include "pico/stdlib.h"

int
main (void)
{
    stdio_init_all();

    sleep_ms(2000);

    printf("Hello, World!\n");

    while (true)
    {
        tight_loop_contents();
    }
}
