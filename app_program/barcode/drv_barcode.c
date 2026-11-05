/**
 * @file drv_barcode.c
 * @brief Barcode sensor DO (GP28): timestamped edges from its interrupt.
 *
 * The edge handler (interrupt context, core 0) writes the queue head; the
 * barcode task (core 0) reads the tail. One producer and one consumer on
 * the same core need no lock: each index is written by one side only.
 */

#include "drv_barcode.h"
#include <bsp/libbsp.h>
#include "barcode_config.h"
#include "../obstacle/drv_gpio_irq.h"

/* Private types, constants, and macros. */
#define QUEUE_MASK (BARCODE_EDGE_QUEUE_SIZE - 1u)

_Static_assert((BARCODE_EDGE_QUEUE_SIZE & QUEUE_MASK) == 0u,
               "Edge queue size must be a power of two");

/* Static data. */
static barcode_edge_t queue[BARCODE_EDGE_QUEUE_SIZE];
static volatile uint32_t head; /* Written by the edge handler only. */
static volatile uint32_t tail; /* Written by the barcode task only. */
static volatile uint32_t overflows;

/* Private function declarations. */
static void sensor_edges (uint32_t events, uint32_t now_us);

/* Public function definitions. */

ER
drv_barcode_init (void)
{
    /* Input with pull-down and Schmitt trigger; the module drives DO. */
    out_w(GPIO_OE_CLR, 1u << BARCODE_PIN);
    out_w(GPIO(BARCODE_PIN), GPIO_IE | GPIO_PDE | GPIO_SHEMITT | GPIO_OD);
    out_w(GPIO_CTRL(BARCODE_PIN), GPIO_CTRL_FUNCSEL_SIO);
    head = 0u;
    tail = 0u;
    return drv_gpio_irq_attach(BARCODE_PIN, sensor_edges);
}

bool
drv_barcode_pop (barcode_edge_t * p_edge)
{
    uint32_t const index = tail;
    if (index == head)
    {
        return false;
    }
    *p_edge = queue[index & QUEUE_MASK];
    tail = index + 1u;
    return true;
}

bool
drv_barcode_black (void)
{
    return gpio_get_val(BARCODE_PIN) == BARCODE_DO_BLACK_LEVEL;
}

uint32_t
drv_barcode_overflows (void)
{
    return overflows;
}

/* Private function definitions. */

/*
 * Interrupt context. One edge gives the new level directly; both edges
 * latched together are a pulse shorter than the interrupt latency, so the
 * level is read back and the scan treats an unchanged level as a glitch.
 */
static void
sensor_edges (uint32_t events, uint32_t now_us)
{
    bool b_high;
    if (DRV_GPIO_IRQ_EDGE_RISE == events)
    {
        b_high = true;
    }
    else if (DRV_GPIO_IRQ_EDGE_FALL == events)
    {
        b_high = false;
    }
    else
    {
        b_high = (gpio_get_val(BARCODE_PIN) != 0u);
    }

    uint32_t const index = head;
    if ((index - tail) >= BARCODE_EDGE_QUEUE_SIZE)
    {
        overflows++;
        return;
    }
    queue[index & QUEUE_MASK] =
        (barcode_edge_t){.time_us = now_us,
                         .b_black = (b_high == (BARCODE_DO_BLACK_LEVEL != 0u))};
    head = index + 1u;
}
