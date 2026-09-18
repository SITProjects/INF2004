#ifndef TEST_PICO_TIME_H
#define TEST_PICO_TIME_H

#include <stdbool.h>
#include <stdint.h>

typedef struct repeating_timer
{
    int unused;
} repeating_timer_t;

uint32_t time_us_32 (void);
void busy_wait_us_32 (uint32_t delay_us);
bool add_repeating_timer_ms (int32_t delay_ms,
                             bool (*callback)(repeating_timer_t *),
                             void * user_data, repeating_timer_t * timer);

#endif
