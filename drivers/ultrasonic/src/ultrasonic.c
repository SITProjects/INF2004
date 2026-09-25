/**
 * @file ultrasonic.c
 * @brief Interrupt-driven ultrasonic pulse measurement.
 * SPDX-License-Identifier: MIT
 */
#include "ultrasonic.h"
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#define ECHO_TIMEOUT_US 23000u
#define PING_INTERVAL_US 40000u
#define CM_PER_US 0.01715f

static volatile uint32_t echo_start_us;
static volatile uint32_t echo_pulse_us;
static volatile uint32_t trigger_start_us;
static volatile bool measurement_ready;
static volatile bool measurement_pending;
static volatile bool echo_active;
static bool has_triggered;
static int ultrasonic_echo_pin;
static int ultrasonic_trig_pin;

static void ultrasonic_isr (void);
static void invalidate_measurement (uint32_t now_us);

int
ultrasonic_init (int trig_pin, int echo_pin)
{
    if (trig_pin < 0 || trig_pin >= (int)NUM_BANK0_GPIOS || echo_pin < 0 ||
        echo_pin >= (int)NUM_BANK0_GPIOS || trig_pin == echo_pin)
    {
        return 1;
    }

    ultrasonic_echo_pin = echo_pin;
    ultrasonic_trig_pin = trig_pin;
    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);
    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);

    gpio_add_raw_irq_handler(echo_pin, ultrasonic_isr);
    gpio_acknowledge_irq(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                         true);
    return 0;
}

bool
ultrasonic_trigger (void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint32_t now_us = time_us_32();
    invalidate_measurement(now_us);
    if (measurement_pending || measurement_ready ||
        (has_triggered &&
         (uint32_t)(now_us - trigger_start_us) < PING_INTERVAL_US))
    {
        restore_interrupts(irq_state);
        return false;
    }

    gpio_acknowledge_irq(ultrasonic_echo_pin,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    trigger_start_us = now_us;
    has_triggered = true;
    echo_active = false;
    measurement_pending = true;

    /* Only the 12 us trigger pulse blocks; echo reception is asynchronous. */
    gpio_put(ultrasonic_trig_pin, 0);
    busy_wait_us_32(2);
    gpio_put(ultrasonic_trig_pin, 1);
    busy_wait_us_32(10);
    gpio_put(ultrasonic_trig_pin, 0);
    restore_interrupts(irq_state);
    return true;
}

bool
ultrasonic_read (float * distance_cm)
{
    uint32_t irq_state = save_and_disable_interrupts();
    invalidate_measurement(time_us_32());
    if (!measurement_ready)
    {
        restore_interrupts(irq_state);
        return false;
    }

    uint32_t pulse_us = echo_pulse_us;
    measurement_ready = false;
    restore_interrupts(irq_state);
    *distance_cm = pulse_us ? (float)pulse_us * CM_PER_US : -1.0f;
    return true;
}

static void
ultrasonic_isr (void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint32_t events = gpio_get_irq_event_mask(ultrasonic_echo_pin);
    uint32_t now_us = time_us_32();
    gpio_acknowledge_irq(ultrasonic_echo_pin, events);
    invalidate_measurement(now_us);

    if ((events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) ==
        (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL))
    {
        /* Edge ordering is unknown; let this measurement expire. */
        echo_active = false;
    }
    else if (measurement_pending && (events & GPIO_IRQ_EDGE_RISE))
    {
        echo_start_us = now_us;
        echo_active = true;
    }
    else if (measurement_pending && echo_active &&
             (events & GPIO_IRQ_EDGE_FALL))
    {
        echo_pulse_us = now_us - echo_start_us;
        measurement_ready = true;
        measurement_pending = false;
        echo_active = false;
    }
    restore_interrupts(irq_state);
}

/* Caller holds the interrupt lock. Unsigned subtraction handles timer wrap. */
static void
invalidate_measurement (uint32_t now_us)
{
    if (measurement_pending &&
        (uint32_t)(now_us - trigger_start_us) >= ECHO_TIMEOUT_US)
    {
        measurement_pending = false;
        echo_active = false;
        echo_pulse_us = 0;
        measurement_ready = true;
    }
}
