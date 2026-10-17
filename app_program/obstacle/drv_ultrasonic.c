/**
 * @file drv_ultrasonic.c
 * @brief Interrupt-driven ultrasonic pulse measurement.
 *
 * Pins use the board helpers in bsp/libbsp.h; the echo edges are timed
 * with the RP2040's free-running 1 MHz TIMER. The echo interrupt handler
 * and the calling task share state under an interrupt-preserving spinlock,
 * which also excludes the other core on the dual-core build.
 *
 * SPDX-License-Identifier: MIT
 */
#include "drv_ultrasonic.h"
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tk/syslib.h>
#include <bsp/libbsp.h>

/* Private types, constants, and macros. */
#define ECHO_TIMEOUT_US 23000u
#define PING_INTERVAL_US 40000u
#define CM_PER_US 0.01715f
#define TRIG_SETTLE_US 2u
#define TRIG_PULSE_US 10u

/* Bank-0 GPIOs on the RP2040. */
#define GPIO_COUNT 30

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
#define IRQ_EDGE_FALL 0x4u
#define IRQ_EDGE_RISE 0x8u
#define IRQ_EDGES (IRQ_EDGE_FALL | IRQ_EDGE_RISE)

/* Atomic bit-set alias of an RP2040 peripheral register. */
#define REG_ALIAS_SET 0x2000u

/* Static data. */
static volatile uint32_t echo_start_us;
static volatile uint32_t echo_pulse_us;
static volatile uint32_t trigger_start_us;
static volatile bool measurement_ready;
static volatile bool measurement_pending;
static volatile bool echo_active;
static bool has_triggered;
static UINT ultrasonic_echo_pin;
static UINT ultrasonic_trig_pin;
static UW irq_reg_offset; /* Byte offset of the echo pin's IRQ registers. */
static UW irq_shift;      /* Bit position of the echo pin in them. */
#if TK_SUPPORT_SMP
static T_SPLOCK ultrasonic_lock;
#endif

/* Private function declarations. */
static void ultrasonic_isr (UINT intno);
static void invalidate_measurement (uint32_t now_us);
static uint32_t now_us (void);
static void acknowledge_edges (void);
static UINT lock (void);
static void unlock (UINT intsts);

/* Public function definitions. */

int
ultrasonic_init (int trig_pin, int echo_pin)
{
    if (trig_pin < 0 || trig_pin >= GPIO_COUNT || echo_pin < 0 ||
        echo_pin >= GPIO_COUNT || trig_pin == echo_pin)
    {
        return 1;
    }
#if TK_SUPPORT_SMP
    /* Processor 1 is RP2040 core 0; its NVIC receives the echo IRQ. */
    if (tk_get_prc() != 1)
    {
        return 1;
    }
#endif

    ultrasonic_echo_pin = (UINT)echo_pin;
    ultrasonic_trig_pin = (UINT)trig_pin;
    irq_reg_offset = IRQ_REG_BYTES * (ultrasonic_echo_pin / GPIOS_PER_IRQ_REG);
    irq_shift = IRQ_BITS_PER_GPIO * (ultrasonic_echo_pin % GPIOS_PER_IRQ_REG);
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&ultrasonic_lock);
#endif

    if ((gpio_set_pin(ultrasonic_trig_pin, GPIO_MODE_OUT) < E_OK) ||
        (gpio_set_val(ultrasonic_trig_pin, 0u) < E_OK) ||
        (gpio_set_pin(ultrasonic_echo_pin, GPIO_MODE_IN) < E_OK))
    {
        return 1;
    }

    T_DINT dint;
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)ultrasonic_isr;
    if (tk_def_int(INTNO_IO_BANK0, &dint) < E_OK)
    {
        return 1;
    }

    acknowledge_edges();
    out_w(IO_BANK0_PROC0_INTE0 + irq_reg_offset + REG_ALIAS_SET,
          IRQ_EDGES << irq_shift);
    EnableInt(INTNO_IO_BANK0, INTPRI_IO_BANK0);
    return 0;
}

bool
ultrasonic_trigger (void)
{
    UINT intsts = lock();
    uint32_t now = now_us();
    invalidate_measurement(now);
    if (measurement_pending || measurement_ready ||
        (has_triggered &&
         (uint32_t)(now - trigger_start_us) < PING_INTERVAL_US))
    {
        unlock(intsts);
        return false;
    }

    acknowledge_edges();
    trigger_start_us = now;
    has_triggered = true;
    echo_active = false;
    measurement_pending = true;

    /* Only the 12 us trigger pulse blocks; echo reception is asynchronous. */
    (void)gpio_set_val(ultrasonic_trig_pin, 0u);
    WaitUsec(TRIG_SETTLE_US);
    (void)gpio_set_val(ultrasonic_trig_pin, 1u);
    WaitUsec(TRIG_PULSE_US);
    (void)gpio_set_val(ultrasonic_trig_pin, 0u);
    unlock(intsts);
    return true;
}

bool
ultrasonic_read (float * distance_cm)
{
    UINT intsts = lock();
    invalidate_measurement(now_us());
    if (!measurement_ready)
    {
        unlock(intsts);
        return false;
    }

    uint32_t pulse_us = echo_pulse_us;
    measurement_ready = false;
    unlock(intsts);
    *distance_cm = pulse_us ? (float)pulse_us * CM_PER_US : -1.0f;
    return true;
}

/* Private function definitions. */

static void
ultrasonic_isr (UINT intno)
{
    (void)intno;

    UINT intsts = lock();
    uint32_t events =
        (in_w(IO_BANK0_PROC0_INTS0 + irq_reg_offset) >> irq_shift) & IRQ_EDGES;
    uint32_t now = now_us();
    out_w(IO_BANK0_INTR0 + irq_reg_offset, events << irq_shift);
    invalidate_measurement(now);

    if ((events & IRQ_EDGES) == IRQ_EDGES)
    {
        /* Edge ordering is unknown; let this measurement expire. */
        echo_active = false;
    }
    else if (measurement_pending && (events & IRQ_EDGE_RISE))
    {
        echo_start_us = now;
        echo_active = true;
    }
    else if (measurement_pending && echo_active && (events & IRQ_EDGE_FALL))
    {
        echo_pulse_us = now - echo_start_us;
        measurement_ready = true;
        measurement_pending = false;
        echo_active = false;
    }
    unlock(intsts);
}

/* Caller holds the lock. Unsigned subtraction handles timer wrap. */
static void
invalidate_measurement (uint32_t now)
{
    if (measurement_pending &&
        (uint32_t)(now - trigger_start_us) >= ECHO_TIMEOUT_US)
    {
        measurement_pending = false;
        echo_active = false;
        echo_pulse_us = 0;
        measurement_ready = true;
    }
}

/* Low 32 bits of the RP2040's free-running microsecond timer. */
static uint32_t
now_us (void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

/* Clear latched echo edges. Caller holds the lock or the IRQ is off. */
static void
acknowledge_edges (void)
{
    out_w(IO_BANK0_INTR0 + irq_reg_offset, IRQ_EDGES << irq_shift);
}

/*
 * Exclude the echo interrupt and, on the dual-core build, the other core.
 * Returns the state to pass to unlock().
 */
static UINT
lock (void)
{
    UINT intsts;
#if TK_SUPPORT_SMP
    (void)ISpinLock(&ultrasonic_lock, &intsts);
#else
    DI(intsts);
#endif
    return intsts;
}

static void
unlock (UINT intsts)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&ultrasonic_lock, intsts);
#else
    EI(intsts);
#endif
}
