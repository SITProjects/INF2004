/**
 * @file drv_gpio_irq.c
 * @brief Share the one bank-0 GPIO interrupt (IO_IRQ_BANK0) between pins.
 *
 * SPDX-License-Identifier: MIT
 */
#include "drv_gpio_irq.h"
#include <stdbool.h>
#include <tk/smp.h>
#include <tk/syslib.h>

/* Private types, constants, and macros. */

/* Bank-0 GPIOs on the RP2040. */
#define GPIO_COUNT 30u

/* IO_IRQ_BANK0: one interrupt for every bank-0 GPIO (RP2040 datasheet). */
#define INTNO_IO_BANK0 13u
/* Same NVIC priority as the kernel's sample device drivers. */
#define INTPRI_IO_BANK0 2

/*
 * IO_BANK0 interrupt registers. Each 32-bit register covers 8 GPIOs with
 * 4 bits per GPIO: level low, level high, edge low, edge high. INTR holds
 * the raw edge latches (write 1 to clear); PROC0_INTE and PROC0_INTS are
 * core 0's enable and masked status.
 */
#define IO_BANK0_INTR0 (IO_BANK0_BASE + 0x0F0u)
#define IO_BANK0_PROC0_INTE0 (IO_BANK0_BASE + 0x100u)
#define IO_BANK0_PROC0_INTS0 (IO_BANK0_BASE + 0x120u)
#define GPIOS_PER_IRQ_REG 8u
#define IRQ_BITS_PER_GPIO 4u
#define IRQ_REG_BYTES 4u

/* Atomic bit-set alias of an RP2040 peripheral register. */
#define REG_ALIAS_SET 0x2000u

typedef struct
{
    drv_gpio_irq_handler_t handler;
    uint32_t reg_offset; /* Byte offset of the pin's IRQ registers. */
    uint32_t shift;      /* Bit position of the pin in them. */
} attached_pin_t;

/* Static data. */
/* Written with interrupts off before the pin's edges are enabled. */
static attached_pin_t pins[DRV_GPIO_IRQ_MAX_PINS];
static volatile uint32_t pin_count;
static volatile bool b_defined;

/* Private function declarations. */
static void gpio_isr (UINT intno);
static uint32_t find_pin (uint32_t reg_offset, uint32_t shift);

/* Public function definitions. */

ER
drv_gpio_irq_attach (uint32_t pin, drv_gpio_irq_handler_t handler)
{
    if ((pin >= GPIO_COUNT) || (NULL == handler))
    {
        return E_PAR;
    }
#if TK_SUPPORT_SMP
    /* Processor 1 is RP2040 core 0; its NVIC receives the interrupt. */
    if (tk_get_prc() != 1)
    {
        return E_CTX;
    }
#endif
    uint32_t const reg_offset = IRQ_REG_BYTES * (pin / GPIOS_PER_IRQ_REG);
    uint32_t const shift = IRQ_BITS_PER_GPIO * (pin % GPIOS_PER_IRQ_REG);

    if (!b_defined)
    {
        /* Two first callers at once only define the same handler twice. */
        T_DINT dint;
        dint.intatr = TA_HLNG;
        dint.inthdr = (FP)gpio_isr;
        ER const defined = tk_def_int(INTNO_IO_BANK0, &dint);
        if (defined < E_OK)
        {
            return defined;
        }
        b_defined = true;
    }

    UINT intsts;
    DI(intsts);
    ER result = E_OK;
    uint32_t const index = find_pin(reg_offset, shift);
    if ((index == pin_count) && (pin_count >= DRV_GPIO_IRQ_MAX_PINS))
    {
        result = E_LIMIT;
    }
    if (result >= E_OK)
    {
        /* Attaching a pin again replaces its handler. */
        pins[index] = (attached_pin_t){
            .handler = handler, .reg_offset = reg_offset, .shift = shift};
        if (index == pin_count)
        {
            pin_count++;
        }
        /* Drop edges latched before now, then enable this pin's edges. */
        out_w(IO_BANK0_INTR0 + reg_offset, DRV_GPIO_IRQ_EDGES << shift);
        out_w(IO_BANK0_PROC0_INTE0 + reg_offset + REG_ALIAS_SET,
              DRV_GPIO_IRQ_EDGES << shift);
    }
    EI(intsts);
    if (E_OK == result)
    {
        EnableInt(INTNO_IO_BANK0, INTPRI_IO_BANK0);
    }
    return result;
}

/* Private function definitions. */

/* Read and clear each attached pin's edges, then call its handler. */
static void
gpio_isr (UINT intno)
{
    (void)intno;

    uint32_t const count = pin_count;
    for (uint32_t i = 0u; i < count; i++)
    {
        attached_pin_t const * p_pin = &pins[i];
        uint32_t const events =
            (in_w(IO_BANK0_PROC0_INTS0 + p_pin->reg_offset) >> p_pin->shift) &
            DRV_GPIO_IRQ_EDGES;
        if (events != 0u)
        {
            uint32_t const now = (uint32_t)in_w(TIMER_TIMERAWL);
            out_w(IO_BANK0_INTR0 + p_pin->reg_offset, events << p_pin->shift);
            p_pin->handler(events, now);
        }
    }
}

/* Index of the attached pin, or pin_count. Caller has interrupts off. */
static uint32_t
find_pin (uint32_t reg_offset, uint32_t shift)
{
    uint32_t i = 0u;
    while ((i < pin_count) &&
           ((pins[i].reg_offset != reg_offset) || (pins[i].shift != shift)))
    {
        i++;
    }
    return i;
}
