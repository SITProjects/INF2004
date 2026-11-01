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
#include "motion_move.h"
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

#if MOTION_MOVE_TESTS
/* MOT-U08..U10: floor moves, then a held wheel on the lifted car. */
#define MOVE_START_DELAY_MS 5000
#define MOVE_TIMEOUT_MS 15000
#define MOVE_SETTLE_MS 500
#define FORWARD_TEST_MM 1000u
#define BACKWARD_TEST_MM 100u
#define STALL_START_DELAY_MS 10000
#endif /* MOTION_MOVE_TESTS */

/* Private function declarations. */
static BOOL slice_braked (UW pin);
static BOOL encoders_steady (void);
static BOOL move_idle_at_rest (void);
#if MOTION_MANUAL_TESTS
static void print_counts_while_turning (void);
static void spin_each_wheel (void);
static UW pin_level (UW levels, UW pin);
#endif
#if MOTION_MOVE_TESTS
static void run_move (char const * p_id, uint16_t distance_mm, bool b_forward);
static BOOL hold_wheel_until_stall (void);
static INT update_while_moving (void);
static void update_for (INT duration_ms);
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

    /* MOT-U07: idle at rest; a 0 mm move is refused. */
    TEST_CHECK("MOT-U07", move_idle_at_rest(),
               "moves start stopped, odometry holds at rest, 0 mm refused");

#if MOTION_MANUAL_TESTS
    /* MOT-U06: manual; see testing/motion/TEST_LOG.md. */
    spin_each_wheel();

    /* MOT-U04, MOT-U05: manual; see testing/motion/TEST_LOG.md. */
    print_counts_while_turning();
#endif

#if MOTION_MOVE_TESTS
    /* MOT-U08..U10: manual; see testing/motion/TEST_LOG.md. */
    run_move("MOT-U08", FORWARD_TEST_MM, true);
    run_move("MOT-U09", BACKWARD_TEST_MM, false);
    TEST_CHECK("MOT-U10", hold_wheel_until_stall(),
               "held wheel stalls, brakes, refuses moves until stopped");
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

/* STOPPED and not busy; distance and speed hold over a rest period. */
static BOOL
move_idle_at_rest (void)
{
    motion_odometry_t before;
    motion_odometry_t after;

    if ((motion_move_stop() != MOTION_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_get_odometry(&before) != MOTION_OK) ||
        (tk_dly_tsk(REST_CHECK_MS) != E_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_get_odometry(&after) != MOTION_OK))
    {
        return FALSE;
    }
    return (MOTION_STATE_STOPPED == motion_move_get_state()) &&
           !motion_move_is_busy() &&
           (before.distance_mm == after.distance_mm) &&
           (0 == after.speed_mm_s) &&
           (motion_move_forward(0u) == MOTION_ERR_PARAM) &&
           !motion_move_is_busy() && slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
           slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
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

#if MOTION_MOVE_TESTS
/* Run one move on the floor and print the odometry to compare with a
 * ruler. */
static void
run_move (char const * p_id, uint16_t distance_mm, bool b_forward)
{
    motion_odometry_t start;
    motion_odometry_t end;

    tm_printf((UB *)"[TEST] %s car on the floor: mark its position; it "
                    "drives %s %d mm in %d s\n",
              p_id, b_forward ? "FORWARD" : "BACKWARD", (INT)distance_mm,
              MOVE_START_DELAY_MS / 1000);
    if (tk_dly_tsk(MOVE_START_DELAY_MS) != E_OK)
    {
        return;
    }

    (void)motion_move_update();
    (void)motion_move_get_odometry(&start);
    motion_status_t status = b_forward ? motion_move_forward(distance_mm)
                                       : motion_move_backward(distance_mm);
    if (status != MOTION_OK)
    {
        tm_printf((UB *)"[TEST] %s move refused (%d)\n", p_id, (INT)status);
        return;
    }

    INT moving_ms = update_while_moving();
    motion_state_t end_state = motion_move_get_state();
    (void)motion_move_stop();
    /* Keep counting while the car coasts to rest. */
    update_for(MOVE_SETTLE_MS);
    (void)motion_move_get_odometry(&end);
    tm_printf((UB *)"[TEST] %s state %d after %d ms; odometry %d mm "
                    "(target %d): ruler must agree within 3 percent\n",
              p_id, (INT)end_state, moving_ms,
              (INT)(end.distance_mm - start.distance_mm),
              b_forward ? (INT)distance_mm : -(INT)distance_mm);
}

/* Drive the lifted car with the left wheel held; check the stall. */
static BOOL
hold_wheel_until_stall (void)
{
    tm_printf((UB *)"[TEST] MOT-U10 lift the car and hold the LEFT wheel "
                    "still: both wheels are driven forward in %d s\n",
              STALL_START_DELAY_MS / 1000);
    if ((tk_dly_tsk(STALL_START_DELAY_MS) != E_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_forward(FORWARD_TEST_MM) != MOTION_OK))
    {
        return FALSE;
    }

    INT moving_ms = update_while_moving();
    BOOL b_stalled = (MOTION_STATE_STALLED == motion_move_get_state());
    BOOL b_braked = slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
                    slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
    BOOL b_refused =
        (motion_move_forward(FORWARD_TEST_MM) == MOTION_ERR_STALLED);
    BOOL b_cleared = (motion_move_stop() == MOTION_OK) &&
                     (MOTION_STATE_STOPPED == motion_move_get_state());
    tm_printf((UB *)"[TEST] MOT-U10 stalled %d after %d ms (expect about "
                    "%d), braked %d, refused %d, cleared %d\n",
              (INT)b_stalled, moving_ms, (INT)MOTION_STALL_TIMEOUT_MS,
              (INT)b_braked, (INT)b_refused, (INT)b_cleared);
    return b_stalled && b_braked && b_refused && b_cleared;
}

/* Update every period until the move ends or times out; return its ms. */
static INT
update_while_moving (void)
{
    INT elapsed = 0;

    while (motion_move_is_busy() && (elapsed < MOVE_TIMEOUT_MS))
    {
        if ((tk_dly_tsk((TMO)MOTION_UPDATE_PERIOD_MS) != E_OK) ||
            (motion_move_update() != MOTION_OK))
        {
            break;
        }
        elapsed += (INT)MOTION_UPDATE_PERIOD_MS;
    }
    return elapsed;
}

/* Keep odometry current for @p duration_ms with no move running. */
static void
update_for (INT duration_ms)
{
    for (INT elapsed = 0; elapsed < duration_ms;
         elapsed += (INT)MOTION_UPDATE_PERIOD_MS)
    {
        (void)tk_dly_tsk((TMO)MOTION_UPDATE_PERIOD_MS);
        (void)motion_move_update();
    }
}
#endif /* MOTION_MOVE_TESTS */

#endif /* APP_TEST */
