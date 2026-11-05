/**
 * @file line_follow.c
 * @brief Line-following subsystem: two digital IR sensors and decisions.
 *
 * The line task samples both DO pins every LINE_SAMPLE_MS, runs the
 * tracking decisions (line_track.c) with motion's shared odometry, and
 * submits each request to the motion task, which owns the motors. It
 * never calls the motor driver directly. It prints each mode change and
 * junction, and publishes them over MQTT in MQTT builds.
 * IR_CALIBRATION_ONLY only prints the sensor levels, for setting each
 * module's potentiometer so DO flips at the tape edge.
 */

#include "line_follow.h"
#include "drv_line_sensor.h"
#include "line_follow_config.h"
#include "line_track.h"
#include "../motion/motion.h"
#include "../motion/motion_command.h"
#include "../motion/motion_command_config.h"
#if TM_MQTT
#include "../communication/telemetry_queue.h"
#endif

/* Private types, constants, and macros. */
#define CALIBRATION_PRINT_MS 1000u
/* "line JUNCTION t=%d h=%d d=%d" with three 11-character values fits. */
#define REPORT_TEXT_SIZE 64u

/* Static data. */
ID line_follow_tid_sense;
static line_track_t track;
static int32_t last_distance_mm;
static int32_t last_heading_deg;

/* Private function declarations. */
static void sense_task (INT stacd, void * p_exinf);
static line_track_input_t take_input (void);
#if IR_CALIBRATION_ONLY
static void calibration_loop (void);
#else
static void report (line_request_t const * p_request,
                    line_track_input_t const * p_in);
#endif

/* Public function definitions. */

ER
line_follow_init (void)
{
    /* No kernel objects; the line task sets up its pins itself. */
    return E_OK;
}

ER
line_follow_start (void)
{
    line_follow_tid_sense = app_start_task(sense_task, LINE_FOLLOW_PRI_SENSE, 0,
                                           "line_follow_task");
    return (line_follow_tid_sense < E_OK) ? line_follow_tid_sense : E_OK;
}

/* Private function definitions. */

static void
sense_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    drv_line_sensor_init();
#if IR_CALIBRATION_ONLY
    calibration_loop();
#else
#if !APP_TEST && MOTION_COMMAND_ACTUATION_ENABLED
    /* Start sensing after the same preparation delay as the motor owner. */
    (void)tk_dly_tsk(MOTION_COMMAND_START_DELAY_MS);
#endif
    tm_printf((UB *)"line_follow: digital IR line sensing started. "
                    "Submitting requests to motion.\n");

    line_track_input_t in = take_input();
    line_track_init(&track, &in);
    for (;;)
    {
        in = take_input();
        line_request_t const request = line_track_update(&track, &in);
        ER const submission = motion_command_submit(&request);
        if ((submission < E_OK) && (submission != E_TMOUT))
        {
            app_check(submission, "line motion submission");
        }
        report(&request, &in);

        /* A relative delay: late samples are never replayed. */
        (void)tk_dly_tsk(LINE_SAMPLE_MS);
    }
#endif
}

/* One sample of the sensors plus motion's state. */
static line_track_input_t
take_input (void)
{
    line_sensor_reading_t const reading = drv_line_sensor_read();
    line_track_input_t in = {
        .b_left_black = reading.b_left_black,
        .b_right_black = reading.b_right_black,
        .b_control = motion_line_control_active(),
        .b_turning = (ROBOT_MOTION_MODE_TURN == motion_get_status().mode)};
    motion_shared_odometry_t odometry;
    if (motion_get_odometry(&odometry))
    {
        last_distance_mm = odometry.distance_mm;
        last_heading_deg = odometry.heading_deg;
    }
    /* After a torn read the last values stand; the next sample retries. */
    in.distance_mm = last_distance_mm;
    in.heading_deg = last_heading_deg;
    return in;
}

#if IR_CALIBRATION_ONLY
/* Print both levels on every change and once a second. */
static void
calibration_loop (void)
{
    tm_printf(
        (UB *)"line_follow: DO calibration; motors not started. Turn each "
              "potentiometer until DO flips at the tape edge (1 = black)\n");
    line_sensor_reading_t last = drv_line_sensor_read();
    uint32_t quiet_ms = CALIBRATION_PRINT_MS;
    for (;;)
    {
        line_sensor_reading_t const now = drv_line_sensor_read();
        bool const b_changed = (now.b_left_black != last.b_left_black) ||
                               (now.b_right_black != last.b_right_black);
        if (b_changed || (quiet_ms >= CALIBRATION_PRINT_MS))
        {
            tm_printf((UB *)"LINE L=%u R=%u%s\n", (UINT)now.b_left_black,
                      (UINT)now.b_right_black, b_changed ? " *" : "");
            quiet_ms = 0u;
        }
        last = now;
        quiet_ms += LINE_SAMPLE_MS;
        (void)tk_dly_tsk(LINE_SAMPLE_MS);
    }
}
#else
/* Print and publish mode changes and junctions. */
static void
report (line_request_t const * p_request, line_track_input_t const * p_in)
{
    static line_mode_t last_mode = LINE_MODE_TRACK;
    static bool b_last_junction;
    bool const b_mode = (p_request->mode != last_mode);
    bool const b_junction = p_request->b_junction && !b_last_junction;
    last_mode = p_request->mode;
    b_last_junction = p_request->b_junction;
    if (!b_mode && !b_junction)
    {
        return;
    }

    char text[REPORT_TEXT_SIZE];
    (void)tm_sprintf((UB *)text, (const UB *)"line %s t=%d h=%d d=%d",
                     b_junction ? "JUNCTION"
                                : line_track_mode_name(p_request->mode),
                     (INT)p_request->target_heading_deg, (INT)p_in->heading_deg,
                     (INT)p_in->distance_mm);
    tm_printf((UB *)"%s\n", text);
#if TM_MQTT
    uint32_t sequence = 0u;
    SYSTIM now;
    uint32_t const now_ms = (tk_get_otm(&now) == E_OK) ? (uint32_t)now.lo : 0u;
    /* A full or busy queue drops it: the console line is the record. */
    (void)telemetry_queue_push(now_ms, text, &sequence);
#endif
}
#endif
