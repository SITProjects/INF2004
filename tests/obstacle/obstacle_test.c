#include "obstacle.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"

static uint32_t now_us;
static uint32_t irq_events;
static bool interrupts_disabled;
static unsigned int trigger_count;
static void (*echo_handler)(void);
static bool (*timer_callback)(repeating_timer_t *);
static void (*on_restore)(void);
static repeating_timer_t timer;

uint32_t
time_us_32 (void)
{
    return now_us;
}

void
busy_wait_us_32 (uint32_t delay_us)
{
    now_us += delay_us;
}

bool
add_repeating_timer_ms (int32_t delay_ms, bool (*callback)(repeating_timer_t *),
                        void * user_data, repeating_timer_t * timer_ptr)
{
    assert(delay_ms == -60);
    assert(user_data == NULL);
    assert(timer_ptr == &timer);
    timer_callback = callback;
    return true;
}

void
gpio_init (unsigned int pin)
{
    assert(pin == 2 || pin == 3);
}

void
gpio_set_dir (unsigned int pin, bool out)
{
    assert((pin == 2 && !out) || (pin == 3 && out));
}

void
gpio_put (unsigned int pin, bool value)
{
    assert(pin == 3);
    if (value)
    {
        ++trigger_count;
    }
}

void
gpio_add_raw_irq_handler (unsigned int pin, void (*handler)(void))
{
    assert(pin == 2);
    echo_handler = handler;
}

void
gpio_set_irq_enabled (unsigned int pin, uint32_t events, bool enabled)
{
    assert(pin == 2);
    assert(events == (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL));
    assert(enabled);
}

uint32_t
gpio_get_irq_event_mask (unsigned int pin)
{
    assert(pin == 2);
    return irq_events;
}

void
gpio_acknowledge_irq (unsigned int pin, uint32_t events)
{
    assert(pin == 2);
    irq_events &= ~events;
}

uint32_t
save_and_disable_interrupts (void)
{
    uint32_t previous = interrupts_disabled;
    interrupts_disabled = true;
    return previous;
}

void
restore_interrupts (uint32_t state)
{
    assert(interrupts_disabled);
    interrupts_disabled = state;
    if (!interrupts_disabled && on_restore != NULL)
    {
        void (*callback)(void) = on_restore;
        on_restore = NULL;
        callback();
    }
}

static void
edge (uint32_t events, uint32_t elapsed_us)
{
    assert(!interrupts_disabled);
    now_us += elapsed_us;
    irq_events |= events;
    echo_handler();
    assert(irq_events == 0);
    assert(!interrupts_disabled);
}

static void
trigger (void)
{
    assert(timer_callback(&timer));
    assert(!interrupts_disabled);
}

static void
echo (uint32_t pulse_us)
{
    edge(GPIO_IRQ_EDGE_RISE, 100);
    edge(GPIO_IRQ_EDGE_FALL, pulse_us);
}

static void
assert_close (void)
{
    assert(is_obstacle_detected());
    assert(obstacle_get_distance_cm() > 1.7f);
    assert(obstacle_get_distance_cm() < 1.8f);
}

static void
assert_unavailable (void)
{
    assert(!is_obstacle_detected());
    assert(obstacle_get_distance_cm() == -1.0f);
}

static void
finish_next_echo (void)
{
    edge(GPIO_IRQ_EDGE_FALL, 1000);
}

int
main (void)
{
    assert(obstacle_init(3, 2, &timer) == 0);
    assert_unavailable();
    assert(!obstacle_update());

    /* Unsolicited edges must not become measurements. */
    edge(GPIO_IRQ_EDGE_FALL, 100);
    assert(!obstacle_update());
    assert_unavailable();

    trigger();
    echo(100);
    assert(obstacle_update());
    assert_close();
    assert(!obstacle_update());
    trigger();
    echo(100);
    assert(!obstacle_update());
    assert_close();

    /* No echo clears a previous close reading exactly at the deadline. */
    uint32_t started = now_us;
    trigger();
    now_us = started + 29999;
    assert(!obstacle_update());
    assert_close();
    now_us = started + 30000;
    assert(!obstacle_update());
    assert_unavailable();

    /* A valid far reading clears detection and allows re-entry. */
    trigger();
    echo(100);
    assert(obstacle_update());
    trigger();
    echo(1000);
    assert(!obstacle_update());
    assert(!is_obstacle_detected());
    assert(obstacle_get_distance_cm() > 17.0f);

    /* A missing fall and late edges must expire, too. */
    trigger();
    echo(100);
    assert(obstacle_update());
    trigger();
    edge(GPIO_IRQ_EDGE_RISE, 100);
    edge(GPIO_IRQ_EDGE_FALL, 30000);
    assert(!obstacle_update());
    assert_unavailable();

    /* Unmatched and coalesced edges must not replace the last valid sample. */
    trigger();
    echo(100);
    assert(obstacle_update());
    trigger();
    edge(GPIO_IRQ_EDGE_FALL, 100);
    assert(!obstacle_update());
    assert_close();
    edge(GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 100);
    assert(!obstacle_update());
    assert_close();
    now_us += 30000;
    assert(!obstacle_update());
    assert_unavailable();

    /* Timer-driven retries preserve the timeout even if polling is delayed. */
    trigger();
    echo(100);
    assert(obstacle_update());
    trigger();
    now_us += 60000;
    trigger();
    assert(!obstacle_update());
    assert_unavailable();
    echo(100);
    assert(obstacle_update());

    /* An early trigger must not reset an in-flight measurement's deadline. */
    started = now_us;
    trigger();
    unsigned int count = trigger_count;
    trigger();
    assert(trigger_count == count);
    now_us = started + 30000;
    assert(!obstacle_update());
    assert_unavailable();

    /* Unsigned elapsed-time calculations support the 32-bit clock wrapping. */
    now_us = UINT32_MAX - 200;
    trigger();
    echo(100);
    assert(obstacle_update());
    assert_close();
    now_us = UINT32_MAX - 100;
    trigger();
    now_us += 30000;
    assert(!obstacle_update());
    assert_unavailable();

    /* An echo arriving at the handoff remains available for the next poll. */
    trigger();
    echo(100);
    trigger();
    edge(GPIO_IRQ_EDGE_RISE, 100);
    on_restore = finish_next_echo;
    assert(obstacle_update());
    assert(on_restore == NULL);
    assert_close();
    assert(!obstacle_update());
    assert(!is_obstacle_detected());
    assert(obstacle_get_distance_cm() > 17.0f);

    puts("Obstacle regression tests passed");
    return 0;
}
