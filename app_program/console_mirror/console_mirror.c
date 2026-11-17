/**
 * @file console_mirror.c
 * @brief Console line buffer for the MQTT mirror (kernel half).
 *
 * Producers: every tm_snd_dat() call, from any task or handler on either
 * core, appends to the ring under a spin lock (interrupts masked), after
 * the USB console has its copy. Consumer: the radio task takes whole lines
 * with console_mirror_take().
 */

#include "console_mirror.h"

#if TM_MQTT
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>

/* Private types, constants, and macros. */
#define RING_SIZE 16384u /* Bytes; a power of two. */
#define RING_MASK (RING_SIZE - 1u)
#define LINE_MAX 512u /* Longest piece of one line in a batch. */
/* A line starts only with this much room, so lines are kept whole. */
#define LINE_RESERVE 256u
#define STAMP_MAX 12u /* Digits of the uptime in ms. */
#define ONLINE_POLL_MS 100u
#define US_PER_MS 1000u
#define BITS_PER_WORD 32u
#define DECIMAL 10u

_Static_assert((RING_SIZE & RING_MASK) == 0u,
               "Console ring size must be a power of two");
_Static_assert(LINE_RESERVE < RING_SIZE, "Line reserve must fit the ring");
_Static_assert(LINE_MAX + 64u <= CONSOLE_MIRROR_BATCH_MAX,
               "A batch must hold a drop notice and the longest piece");

/* Lines from the radio task about telemetry itself; never mirrored. */
static char const * const skipped_prefixes[] = {
    "telemetry: TX ", "telemetry: ACK ", "telemetry: queued "};

#define SKIPPED_COUNT (sizeof(skipped_prefixes) / sizeof(skipped_prefixes[0]))

/* Static data. */
#if TK_SUPPORT_SMP
static T_SPLOCK ring_lock; /* Zero is unlocked. */
#endif
static char ring[RING_SIZE];
static uint32_t head;           /* Written by producers, under the lock. */
static uint32_t tail;           /* Written by the radio task, under the lock. */
static bool b_mid_line;         /* Producers: the current line has started. */
static bool b_dropping;         /* Producers: dropping until the next '\n'. */
static uint32_t dropped_lines;  /* Producers. */
static uint32_t reported_drops; /* Radio task. */
static volatile bool b_online;  /* Written by the radio task. */

/* Private function declarations. */
void __real_tm_snd_dat (UB const * p_buf, INT size);
void __wrap_tm_snd_dat (UB const * p_buf, INT size);
static UINT ring_lock_enter (void);
static void ring_lock_leave (UINT intsts);
static void put_byte (char c);
static void put_stamp (void);
static uint32_t format_decimal (char * p_out, uint32_t value);
static bool line_skipped (uint32_t start, uint32_t length);

/* Public function definitions. */

/*
 * Every console write lands here (makefile: -Wl,--wrap=tm_snd_dat). The
 * USB console gets the bytes first and unchanged.
 */
void
__wrap_tm_snd_dat (UB const * p_buf, INT size)
{
    __real_tm_snd_dat(p_buf, size);

    UINT const intsts = ring_lock_enter();
    for (INT i = 0; i < size; i++)
    {
        char const c = (char)p_buf[i];
        if ('\r' == c)
        {
            continue;
        }
        if (!b_mid_line)
        {
            b_mid_line = true;
            b_dropping = ((head - tail) > (RING_SIZE - LINE_RESERVE));
            if (b_dropping)
            {
                dropped_lines++;
            }
            else
            {
                put_stamp();
            }
        }
        if (!b_dropping)
        {
            put_byte(c);
        }
        if ('\n' == c)
        {
            b_mid_line = false;
        }
    }
    ring_lock_leave(intsts);
}

uint32_t
console_mirror_take (char * p_out)
{
    UINT intsts = ring_lock_enter();
    uint32_t const end = head;
    uint32_t const drops = dropped_lines;
    ring_lock_leave(intsts);

    uint32_t count = 0u;
    if (drops != reported_drops)
    {
        static char const notice[] = "console_mirror: dropped lines, total ";
        for (uint32_t i = 0u; notice[i] != '\0'; i++)
        {
            p_out[count++] = notice[i];
        }
        count += format_decimal(&p_out[count], drops);
        p_out[count++] = '\n';
        reported_drops = drops;
    }

    /* Whole lines up to LINE_MAX bytes; a longer line goes in pieces, and
     * a line still being printed waits for its '\n'. */
    uint32_t start = tail;
    uint32_t taken = 0u;
    while (start != end)
    {
        uint32_t const available = end - start;
        uint32_t const limit = (available < LINE_MAX) ? available : LINE_MAX;
        uint32_t length = 0u;
        while ((length < limit) && (ring[(start + length) & RING_MASK] != '\n'))
        {
            length++;
        }
        if (length < limit)
        {
            length++; /* Include the '\n'. */
        }
        else if ((limit < LINE_MAX) || (taken != 0u))
        {
            break;
        }
        if ((taken + length) > LINE_MAX)
        {
            break;
        }
        if (!line_skipped(start, length))
        {
            for (uint32_t i = 0u; i < length; i++)
            {
                p_out[count++] = ring[(start + i) & RING_MASK];
            }
        }
        start += length;
        taken += length;
    }

    intsts = ring_lock_enter();
    tail = start;
    ring_lock_leave(intsts);
    return count;
}

void
console_mirror_mark_online (void)
{
    b_online = true;
}

bool
console_mirror_wait_online (uint32_t timeout_ms)
{
    uint32_t waited_ms = 0u;
    while (!b_online && (waited_ms < timeout_ms))
    {
        (void)tk_dly_tsk(ONLINE_POLL_MS);
        waited_ms += ONLINE_POLL_MS;
    }
    return b_online;
}

/* Private function definitions. */

static UINT
ring_lock_enter (void)
{
    UINT intsts;
#if TK_SUPPORT_SMP
    (void)ISpinLock(&ring_lock, &intsts);
#else
    DI(intsts);
#endif
    return intsts;
}

static void
ring_lock_leave (UINT intsts)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&ring_lock, intsts);
#else
    EI(intsts);
#endif
}

/* Producers, under the lock. LINE_RESERVE leaves room for a line's stamp
 * and text, so a full ring only cuts a line longer than the reserve. */
static void
put_byte (char c)
{
    if ((head - tail) < RING_SIZE)
    {
        ring[head & RING_MASK] = c;
        head++;
    }
}

/* Producers, under the lock: the uptime in ms and a space. */
static void
put_stamp (void)
{
    uint32_t high;
    uint32_t low;
    do
    {
        high = (uint32_t)in_w(TIMER_TIMERAWH);
        low = (uint32_t)in_w(TIMER_TIMERAWL);
    } while (high != (uint32_t)in_w(TIMER_TIMERAWH));
    uint64_t const now_us = ((uint64_t)high << BITS_PER_WORD) | low;

    char digits[STAMP_MAX];
    uint32_t const count =
        format_decimal(digits, (uint32_t)(now_us / US_PER_MS));
    for (uint32_t i = 0u; i < count; i++)
    {
        put_byte(digits[i]);
    }
    put_byte(' ');
}

/* Decimal digits of @p value into @p p_out (no terminator); the count. */
static uint32_t
format_decimal (char * p_out, uint32_t value)
{
    char reversed[STAMP_MAX];
    uint32_t count = 0u;
    do
    {
        reversed[count] = (char)('0' + (value % DECIMAL));
        value /= DECIMAL;
        count++;
    } while ((value != 0u) && (count < STAMP_MAX));
    for (uint32_t i = 0u; i < count; i++)
    {
        p_out[i] = reversed[count - 1u - i];
    }
    return count;
}

/* True for a radio-task telemetry line (checked after its stamp). */
static bool
line_skipped (uint32_t start, uint32_t length)
{
    uint32_t text = 0u;
    while ((text < length) && (ring[(start + text) & RING_MASK] != ' '))
    {
        text++;
    }
    text++; /* Past the stamp's space. */
    for (uint32_t p = 0u; p < SKIPPED_COUNT; p++)
    {
        char const * const p_prefix = skipped_prefixes[p];
        uint32_t i = 0u;
        while ((p_prefix[i] != '\0') && ((text + i) < length) &&
               (ring[(start + text + i) & RING_MASK] == p_prefix[i]))
        {
            i++;
        }
        if ('\0' == p_prefix[i])
        {
            return true;
        }
    }
    return false;
}
#endif /* TM_MQTT */
