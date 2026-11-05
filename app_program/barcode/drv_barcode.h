/**
 * @file drv_barcode.h
 * @brief Barcode sensor DO (GP28): timestamped edges from its interrupt.
 */

#ifndef DRV_BARCODE_H
#define DRV_BARCODE_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/* Public types, constants, and macros. */

/** @brief One edge of the barcode sensor. */
typedef struct
{
    uint32_t time_us; /**< Low 32 bits of the 1 MHz TIMER. */
    bool b_black;     /**< Level after the edge. */
} barcode_edge_t;

/* Public function declarations. */

/**
 * @brief Set up GP28 as an input and attach its edge interrupt.
 * @return E_OK, or the error of drv_gpio_irq_attach().
 * @note The barcode task only, on core 0.
 */
ER drv_barcode_init (void);

/**
 * @brief Take the oldest queued edge.
 * @param p_edge Filled when an edge was queued.
 * @return True if @p p_edge was filled.
 * @note The barcode task only (single consumer).
 */
bool drv_barcode_pop (barcode_edge_t * p_edge);

/** @brief The sensor's level now. */
bool drv_barcode_black (void);

/** @brief Edges dropped because the queue was full, since start-up. */
uint32_t drv_barcode_overflows (void);

#endif /* DRV_BARCODE_H */
