/**
 * @file drv_gpio_irq.h
 * @brief Share the one bank-0 GPIO interrupt (IO_IRQ_BANK0) between pins.
 *
 * The RP2040 has a single interrupt for every bank-0 GPIO and the kernel
 * one handler per interrupt, so this dispatcher owns it (TEAM.md section
 * 3, obstacle) and calls each attached pin's handler with its latched
 * edges and a 1 MHz TIMER timestamp. Clients: the ultrasonic echo
 * (obstacle) and the barcode sensor (barcode).
 */

#ifndef DRV_GPIO_IRQ_H
#define DRV_GPIO_IRQ_H

/* Public includes. */
#include <stdint.h>
#include <tk/tkernel.h>

/* Public types, constants, and macros. */

/** Edge bits passed to a handler (IO_BANK0 INTR layout). */
#define DRV_GPIO_IRQ_EDGE_FALL 0x4u
#define DRV_GPIO_IRQ_EDGE_RISE 0x8u
#define DRV_GPIO_IRQ_EDGES (DRV_GPIO_IRQ_EDGE_FALL | DRV_GPIO_IRQ_EDGE_RISE)

/** Pins that may be attached at once. */
#define DRV_GPIO_IRQ_MAX_PINS 4u

/**
 * @brief Handler for one pin's edges, called in interrupt context.
 * @param events DRV_GPIO_IRQ_EDGE_ bits latched since the last call; both
 * set means two edges whose order is unknown.
 * @param now_us Low 32 bits of the 1 MHz TIMER, read after the edges.
 */
typedef void (*drv_gpio_irq_handler_t)(uint32_t events, uint32_t now_us);

/* Public function declarations. */

/**
 * @brief Call @p handler on @p pin's rising and falling edges.
 * @param pin Bank-0 GPIO, already set up as an input by the caller.
 * @param handler Runs in interrupt context: short, no waiting.
 * @return E_OK (attaching a pin again replaces its handler); E_PAR for a
 * bad pin or NULL handler; E_LIMIT when DRV_GPIO_IRQ_MAX_PINS other pins
 * are attached; E_CTX off core 0 on the dual-core build; or a
 * tk_def_int() error.
 * @note Task context on core 0 (TP_PRC1): the interrupt is enabled in
 * that core's NVIC. The first call defines the interrupt.
 */
ER drv_gpio_irq_attach (uint32_t pin, drv_gpio_irq_handler_t handler);

#endif /* DRV_GPIO_IRQ_H */
