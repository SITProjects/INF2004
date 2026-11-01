/**
 * @file motion_test.c
 * @brief On-target tests for the motion subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/motion/logs/ and record the result in
 * testing/motion/TEST_LOG.md. Test IDs: MOT-U01, MOT-U02, ...
 */

#include "motion.h"
#include "../test_report.h"
#include "motion_config.h"
#include "motion_drive.h"
#include "drv_motor.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define PWM_SLICE_STRIDE 0x14u
#define PWM_SLICE_COUNT 8u
#define MOTOR_WRAP_10KHZ (((UW)SYSCLK * 100u) - 1u) /* clk_sys / 10 kHz - 1 */

/* PIO0 CTRL; the encoders run on state machines 2 and 3 (drv_encoder.c). */
#define PIO0_CTRL 0x50200000u
#define ENCODER_SM_ENABLE ((1u << 2) | (1u << 3))

/* SMx_ADDR (program counter) of state machines 2 and 3. */
#define PIO0_SM2_ADDR 0x50200104u
#define PIO0_SM3_ADDR 0x5020011cu

#define REST_CHECK_MS 200

#if MOTION_MANUAL_TESTS
#define TURN_WINDOW_MS 20000
#define TURN_PRINT_MS 500

/* MOT-U06: each wheel alone, forward then backward, at a gentle duty. */
#define SPIN_DUTY 0.4f
#define SPIN_START_DELAY_MS 3000
#define SPIN_MS 1500
#define SPIN_PAUSE_MS 1000

typedef struct
{
    motor_id_t id;
    float duty;
    char const * p_name;
} spin_step_t;
#endif /* MOTION_MANUAL_TESTS */

/* Private function declarations. */
static BOOL slice_braked (UW pin);
static BOOL encoders_steady (void);
#if MOTION_MANUAL_TESTS
static void print_counts_while_turning (void);
static void spin_each_wheel (void);
static UW pin_level (UW levels, UW pin);
#endif

/* Public function definitions. */

void
motion_test (void)
{
    TEST_INIT();

    /* MOT-U01: both motors' slices run at 10 kHz with both inputs low. */
    TEST_CHECK("MOT-U01",
               slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
                   slice_braked(MOTION_RIGHT_MOTOR_PIN_A),
               "both motors run 10 kHz PWM and are braked");

    /* MOT-U03: both encoder state machines run; counts hold at rest. */
    TEST_CHECK("MOT-U03",
               ((in_w(PIO0_CTRL) & ENCODER_SM_ENABLE) == ENCODER_SM_ENABLE) &&
                   encoders_steady(),
               "both encoders count on PIO0 and hold still at rest");

#if MOTION_MANUAL_TESTS
    /* MOT-U06: manual; see testing/motion/TEST_LOG.md. */
    spin_each_wheel();

    /* MOT-U04, MOT-U05: manual; see testing/motion/TEST_LOG.md. */
    print_counts_while_turning();
#endif

    TEST_SUMMARY("motion");
}

/* Private function definitions. */

/* The slice of @p pin is enabled, wraps for 10 kHz, and both levels are 0. */
static BOOL
slice_braked (UW pin)
{
    UW base = PWM_BASE + (PWM_SLICE_STRIDE * ((pin / 2u) % PWM_SLICE_COUNT));

    return ((in_w(base + PWM_CHx_CSR) & PWM_CH_CSR_EN) != 0u) &&
           (in_w(base + PWM_CHx_TOP) == MOTOR_WRAP_10KHZ) &&
           (in_w(base + PWM_CHx_CC) == 0u);
}

/* Count reads succeed and neither count changes over REST_CHECK_MS. */
static BOOL
encoders_steady (void)
{
    motion_counts_t before;
    motion_counts_t after;

    if ((motion_drive_get_counts(&before) != MOTION_OK) ||
        (tk_dly_tsk(REST_CHECK_MS) != E_OK) ||
        (motion_drive_get_counts(&after) != MOTION_OK))
    {
        return FALSE;
    }
    return (before.left_count == after.left_count) &&
           (before.right_count == after.right_count);
}

#if MOTION_MANUAL_TESTS
/* Print both counts while a tester turns the wheels by hand. */
static void
print_counts_while_turning (void)
{
    motion_counts_t counts;

    tm_printf((UB *)"[TEST] MOT-U04/U05 turn each wheel one revolution "
                    "forward, then back, within %d s (expect %d per rev)\n",
              TURN_WINDOW_MS / 1000, (INT)MOTION_ENCODER_TICKS_PER_REV);
    for (INT elapsed = 0; elapsed < TURN_WINDOW_MS; elapsed += TURN_PRINT_MS)
    {
        if ((tk_dly_tsk(TURN_PRINT_MS) != E_OK) ||
            (motion_drive_get_counts(&counts) != MOTION_OK))
        {
            tm_printf((UB *)"[TEST] MOT-U04/U05 encoder read failed\n");
            return;
        }
        /* Raw phase levels and PCs show wiring faults apart from code. */
        UW levels = in_w(GPIO_IN);
        tm_printf((UB *)"[TEST] MOT-U04/U05 left %d right %d  "
                        "pins L%d%d R%d%d  pc %d %d\n",
                  (INT)counts.left_count, (INT)counts.right_count,
                  (INT)pin_level(levels, MOTION_LEFT_ENCODER_PIN_A),
                  (INT)pin_level(levels, MOTION_LEFT_ENCODER_PIN_B),
                  (INT)pin_level(levels, MOTION_RIGHT_ENCODER_PIN_A),
                  (INT)pin_level(levels, MOTION_RIGHT_ENCODER_PIN_B),
                  (INT)in_w(PIO0_SM2_ADDR), (INT)in_w(PIO0_SM3_ADDR));
    }
}

/* Spin each wheel alone, forward then backward, and print its counts. */
static void
spin_each_wheel (void)
{
    static spin_step_t const steps[] = {
        {MOTOR_LEFT, SPIN_DUTY, "LEFT wheel FORWARD"},
        {MOTOR_LEFT, -SPIN_DUTY, "LEFT wheel BACKWARD"},
        {MOTOR_RIGHT, SPIN_DUTY, "RIGHT wheel FORWARD"},
        {MOTOR_RIGHT, -SPIN_DUTY, "RIGHT wheel BACKWARD"},
    };
    motion_counts_t before;
    motion_counts_t after;

    tm_printf((UB *)"[TEST] MOT-U06 keep the car lifted: each wheel spins "
                    "alone in %d s\n",
              SPIN_START_DELAY_MS / 1000);
    if (tk_dly_tsk(SPIN_START_DELAY_MS) != E_OK)
    {
        return;
    }

    for (UW i = 0u; i < (sizeof(steps) / sizeof(steps[0])); i++)
    {
        tm_printf((UB *)"[TEST] MOT-U06 %s\n", steps[i].p_name);
        (void)motion_drive_get_counts(&before);
        motor_set_duty(steps[i].id, steps[i].duty);
        ER er = tk_dly_tsk(SPIN_MS);
        motor_brake(steps[i].id);
        if ((er != E_OK) || (motion_drive_get_counts(&after) != MOTION_OK))
        {
            tm_printf((UB *)"[TEST] MOT-U06 stopped early\n");
            return;
        }
        tm_printf((UB *)"[TEST] MOT-U06   encoder change left %d right %d\n",
                  (INT)(int16_t)(after.left_count - before.left_count),
                  (INT)(int16_t)(after.right_count - before.right_count));
        (void)tk_dly_tsk(SPIN_PAUSE_MS);
    }
}

/* Level (0 or 1) of @p pin in a GPIO_IN snapshot. */
static UW
pin_level (UW levels, UW pin)
{
    return (levels >> pin) & 1u;
}
#endif /* MOTION_MANUAL_TESTS */

#endif /* APP_TEST */
