/**
 * @file drv_encoder.c
 * @brief Quadrature wheel encoder driver counting in PIO0 state machines.
 *
 * GPIO edge interrupts belong to the obstacle subsystem (TEAM.md section
 * 3), and the 10 ms kernel tick is too slow to poll the phases, so a PIO
 * state machine decodes each encoder. It samples both phases every few
 * clk_sys cycles, keeps the count in its Y register, and keeps pushing it
 * to its RX FIFO without blocking.
 *
 * The CYW43 radio driver takes a PIO through the Pico SDK allocator, which
 * tries PIO1 first, so PIO0 stays free (TEAM.md section 3). The allocator
 * cannot see this driver's program, so encoder_init() refuses to load it
 * if any PIO0 state machine is already running, rather than overwrite
 * another program or count nothing.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "drv_encoder.h"
#include <tk/tkernel.h>

/* Private types, constants, and macros. */

/* PIO0 registers (RP2040 datasheet section 3.7). */
#define PIO0_BASE 0x50200000u
#define PIO_CTRL (PIO0_BASE + 0x000u)
#define PIO_FSTAT (PIO0_BASE + 0x004u)
#define PIO_FLEVEL (PIO0_BASE + 0x00cu)
#define PIO_RXF0 (PIO0_BASE + 0x020u)
#define PIO_INSTR_MEM0 (PIO0_BASE + 0x048u)
#define PIO_SM0_CLKDIV (PIO0_BASE + 0x0c8u)
#define PIO_SM_STRIDE 0x18u
#define PIO_REG_BYTES 4u

/* Offsets from a state machine's CLKDIV register. */
#define PIO_SM_EXECCTRL 0x04u
#define PIO_SM_SHIFTCTRL 0x08u
#define PIO_SM_INSTR 0x10u
#define PIO_SM_PINCTRL 0x14u

/* Register fields. */
#define PIO_CTRL_SM_ENABLE_MASK 0xfu
#define PIO_CTRL_SM_RESTART_SHIFT 4u
#define PIO_CTRL_CLKDIV_RESTART_SHIFT 8u
#define PIO_FSTAT_RXEMPTY_SHIFT 8u
#define PIO_FLEVEL_RX_SHIFT 4u
#define PIO_FLEVEL_SM_BITS 8u
#define PIO_FLEVEL_MASK 0xfu
#define PIO_CLKDIV_INT_SHIFT 16u
#define PIO_EXECCTRL_WRAP_TOP_SHIFT 12u
#define PIO_EXECCTRL_WRAP_BOTTOM_SHIFT 7u
#define PIO_SHIFTCTRL_OUT_SHIFTDIR (1u << 19) /* OUT shifts right. */
#define PIO_SHIFTCTRL_FJOIN_RX (1u << 31)
#define PIO_PINCTRL_IN_BASE_SHIFT 15u

/* Atomic set and clear aliases of a peripheral register. */
#define REG_ALIAS_SET 0x2000u
#define REG_ALIAS_CLR 0x3000u

/* Bank-0 GPIOs on the RP2040. */
#define ENCODER_GPIO_COUNT 30u

/* Encoder i runs on state machine ENCODER_FIRST_SM + i; see below. */
#define ENCODER_FIRST_SM 2u

/* A fresh sample is at most 10 PIO cycles away; this bounds the wait. */
#define ENCODER_MAX_SPINS 1000u

/*
 * Quadrature decoder from the Raspberry Pi pico-examples
 * (pio/quadrature_encoder/quadrature_encoder.pio, BSD-3-Clause), assembled
 * with pioasm from pico-sdk 2.2.0. Re-run pioasm after any change to the
 * source below. It must be loaded at address 0, because "mov pc, isr"
 * jumps to (previous state << 2) | new state, where a state is (B << 1) | A.
 * Y counts down when A leads B; encoder_get_count() negates it.
 *
 *     .program quadrature_encoder
 *     .origin 0
 *         jmp update      ; 00 -> 00
 *         jmp decrement   ; 00 -> 01
 *         jmp increment   ; 00 -> 10
 *         jmp update      ; 00 -> 11
 *         jmp increment   ; 01 -> 00
 *         jmp update      ; 01 -> 01
 *         jmp update      ; 01 -> 10
 *         jmp decrement   ; 01 -> 11
 *         jmp decrement   ; 10 -> 00
 *         jmp update      ; 10 -> 01
 *         jmp update      ; 10 -> 10
 *         jmp increment   ; 10 -> 11
 *         jmp update      ; 11 -> 00
 *         jmp increment   ; 11 -> 01
 *     decrement:
 *         jmp y--, update ; 11 -> 10
 *     .wrap_target
 *     update:
 *         mov isr, y      ; 11 -> 11
 *         push noblock
 *         out isr, 2
 *         in pins, 2
 *         mov osr, isr
 *         mov pc, isr
 *     increment:
 *         mov y, ~y
 *         jmp y--, increment_cont
 *     increment_cont:
 *         mov y, ~y
 *     .wrap
 *
 * The program fills slots 0-23. If the SDK allocator ever fell back to
 * PIO0, it would take state machine 0 and the top slots, so the encoders
 * use state machines 2 and 3 to stay clear of it.
 */
#define PROGRAM_LENGTH 24u
#define PROGRAM_WRAP_BOTTOM 15u /* .wrap_target, the "update" label. */
#define PROGRAM_WRAP_TOP 23u    /* .wrap */

static uint16_t const program[PROGRAM_LENGTH] = {
    0x000fu, /*  0: jmp    15         */
    0x000eu, /*  1: jmp    14         */
    0x0015u, /*  2: jmp    21         */
    0x000fu, /*  3: jmp    15         */
    0x0015u, /*  4: jmp    21         */
    0x000fu, /*  5: jmp    15         */
    0x000fu, /*  6: jmp    15         */
    0x000eu, /*  7: jmp    14         */
    0x000eu, /*  8: jmp    14         */
    0x000fu, /*  9: jmp    15         */
    0x000fu, /* 10: jmp    15         */
    0x0015u, /* 11: jmp    21         */
    0x000fu, /* 12: jmp    15         */
    0x0015u, /* 13: jmp    21         */
    0x008fu, /* 14: jmp    y--, 15    */
    0xa0c2u, /* 15: mov    isr, y     */
    0x8000u, /* 16: push   noblock    */
    0x60c2u, /* 17: out    isr, 2     */
    0x4002u, /* 18: in     pins, 2    */
    0xa0e6u, /* 19: mov    osr, isr   */
    0xa0a6u, /* 20: mov    pc, isr    */
    0xa04au, /* 21: mov    y, ~y      */
    0x0097u, /* 22: jmp    y--, 23    */
    0xa04au, /* 23: mov    y, ~y      */
};

/*
 * Executed once before the state machine starts (also from pioasm):
 * clear the count, record the phases' current levels as the previous
 * state so start-up does not count a false edge, then enter the loop.
 */
#define PRIME_LENGTH 4u

static uint16_t const prime[PRIME_LENGTH] = {
    0xe040u, /* set    y, 0       */
    0x4002u, /* in     pins, 2    */
    0xa0e6u, /* mov    osr, isr   */
    0x000fu, /* jmp    15         */
};

typedef struct
{
    int16_t count; /* Last count read. */
    uint8_t sm;
    bool b_inverted;
    bool b_ready;
} encoder_t;

/* Static data. */
static encoder_t encoders[ENCODER_COUNT];
static bool b_program_loaded = false;

/* Private function declarations. */
static bool claim_pio0 (void);
static void release_pio0_reset (void);
static void configure_pin (uint32_t pin);
static void start_state_machine (uint32_t sm, uint32_t pin_a);
static UW sm_base (uint32_t sm);

/* Public function definitions. */

bool
encoder_init (encoder_id_t id, uint32_t pin_a, uint32_t pin_b, bool b_inverted)
{
    if ((id >= ENCODER_COUNT) || encoders[id].b_ready ||
        (pin_a >= ENCODER_GPIO_COUNT) || (pin_b >= ENCODER_GPIO_COUNT) ||
        (pin_b != (pin_a + 1u)))
    {
        return false;
    }

    if (!b_program_loaded)
    {
        if (!claim_pio0())
        {
            return false;
        }
        b_program_loaded = true;
    }

    encoder_t * p_encoder = &encoders[id];
    p_encoder->sm = (uint8_t)(ENCODER_FIRST_SM + (uint32_t)id);
    p_encoder->count = 0;
    p_encoder->b_inverted = b_inverted;
    configure_pin(pin_a);
    configure_pin(pin_b);
    start_state_machine(p_encoder->sm, pin_a);
    p_encoder->b_ready = true;
    return true;
}

int16_t
encoder_get_count (encoder_id_t id)
{
    if ((id >= ENCODER_COUNT) || !encoders[id].b_ready)
    {
        return 0;
    }

    encoder_t * p_encoder = &encoders[id];
    uint32_t sm = p_encoder->sm;
    UW rxf = PIO_RXF0 + (PIO_REG_BYTES * sm);

    /* Samples already queued may be old: discard them. */
    UW queued = (in_w(PIO_FLEVEL) >>
                 (PIO_FLEVEL_RX_SHIFT + (PIO_FLEVEL_SM_BITS * sm))) &
                PIO_FLEVEL_MASK;
    for (UW i = 0u; i < queued; i++)
    {
        (void)in_w(rxf);
    }

    for (uint32_t spin = 0u; spin < ENCODER_MAX_SPINS; spin++)
    {
        if ((in_w(PIO_FSTAT) & (1u << (PIO_FSTAT_RXEMPTY_SHIFT + sm))) == 0u)
        {
            /*
             * Keep the low 16 bits of the 32-bit hardware count. Negate in
             * unsigned arithmetic so no value can overflow.
             */
            uint32_t raw = (uint32_t)in_w(rxf);
            uint16_t low = (uint16_t)(p_encoder->b_inverted ? raw : (0u - raw));
            p_encoder->count = (int16_t)low;
            break;
        }
    }

    /* If no sample arrived, the last count read is the best available. */
    return p_encoder->count;
}

/* Private function definitions. */

/* Load the program, unless PIO0 already runs something else. */
static bool
claim_pio0 (void)
{
    release_pio0_reset();
    if ((in_w(PIO_CTRL) & PIO_CTRL_SM_ENABLE_MASK) != 0u)
    {
        return false;
    }

    for (uint32_t i = 0u; i < PROGRAM_LENGTH; i++)
    {
        out_w(PIO_INSTR_MEM0 + (PIO_REG_BYTES * i), (UW)program[i]);
    }
    return true;
}

/* The kernel releases PIO0 only in Wi-Fi builds (hw_setting.c). */
static void
release_pio0_reset (void)
{
    if ((in_w(RESETS_RESET) & RESETS_RESET_PIO0) != 0u)
    {
        out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_RESET_PIO0);
        while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PIO0) == 0u)
        {
            /* Wait for the block to come out of reset. */
        }
    }
}

/* Input with pull-up, routed to PIO0. The PIO never drives the pin. */
static void
configure_pin (uint32_t pin)
{
    out_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_SHEMITT | GPIO_OD);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_PIO0);
}

/* As pio_sm_init() in the Pico SDK, then prime and enable. */
static void
start_state_machine (uint32_t sm, uint32_t pin_a)
{
    UW base = sm_base(sm);
    UW shiftctrl = PIO_SHIFTCTRL_OUT_SHIFTDIR; /* IN shifts left. */

    out_w(PIO_CTRL + REG_ALIAS_CLR, 1u << sm);
    out_w(base, 1u << PIO_CLKDIV_INT_SHIFT); /* Run at clk_sys. */
    out_w(base + PIO_SM_EXECCTRL,
          (PROGRAM_WRAP_TOP << PIO_EXECCTRL_WRAP_TOP_SHIFT) |
              (PROGRAM_WRAP_BOTTOM << PIO_EXECCTRL_WRAP_BOTTOM_SHIFT));

    /* Changing FJOIN_RX empties both FIFOs. */
    out_w(base + PIO_SM_SHIFTCTRL, shiftctrl | PIO_SHIFTCTRL_FJOIN_RX);
    out_w(base + PIO_SM_SHIFTCTRL, shiftctrl);
    out_w(base + PIO_SM_PINCTRL, pin_a << PIO_PINCTRL_IN_BASE_SHIFT);

    /* Clear the shift registers and the clock divider's phase. */
    out_w(PIO_CTRL + REG_ALIAS_SET,
          (1u << (PIO_CTRL_SM_RESTART_SHIFT + sm)) |
              (1u << (PIO_CTRL_CLKDIV_RESTART_SHIFT + sm)));

    /* The INSTR register executes at once, even with the machine off. */
    for (uint32_t i = 0u; i < PRIME_LENGTH; i++)
    {
        out_w(base + PIO_SM_INSTR, (UW)prime[i]);
    }
    out_w(PIO_CTRL + REG_ALIAS_SET, 1u << sm);
}

/* Address of state machine @p sm's first register, SMx_CLKDIV. */
static UW
sm_base (uint32_t sm)
{
    return PIO_SM0_CLKDIV + (PIO_SM_STRIDE * sm);
}
