/**
 * @file app_main.c
 * @brief START HERE. usermain() below is the program's entry point.
 *
 * How micro T-Kernel starts the code
 * ----------------------------------
 *   reset -> kernel initialises itself -> creates the "initial task"
 *         -> initial task calls usermain()
 *
 * usermain() runs inside that initial task at priority 1 (the highest).
 * On a single core nothing it creates runs until usermain() blocks, but on
 * the dual-core build the OTHER core may start a new task at once. So
 * usermain() works in two phases:
 *   1. every subsystem creates its kernel objects  (<name>_init)
 *   2. every subsystem starts its tasks             (<name>_start)
 * and then puts the initial task to sleep forever.
 * If usermain() RETURNS, the kernel shuts the whole system down.
 *
 * The vehicle: one subsystem per folder
 * -------------------------------------
 *   obstacle/       ultrasonic sensor on a servo, scans for a path
 *   motion/         two H-bridge DC motors, held braked for now
 *   line_follow/    two IR sensors -> line-following commands
 *   terrain/        GY-511 accelerometer, calibration at start-up
 *   communication/  Wi-Fi connection status from the kernel's radio task
 *
 * The subsystems do not exchange data yet; TEAM.md section 5 lists the
 * interfaces once they exist. The subsystem that RECEIVES or OWNS the data
 * creates the kernel object and publishes its ID in its header; the
 * other side just uses it.
 *
 * Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */

/* Public function definitions. */

ID
app_start_task (FP entry, PRI pri, UW prc, char const * p_name)
{
    T_CTSK ctsk = {0};

    ctsk.task = entry;            /* Function the task runs. */
    ctsk.itskpri = pri;           /* Initial priority (1 = highest). */
    ctsk.stksz = TASK_STACK_SIZE; /* The kernel allocates the stack. */
    /* Written in C (TA_HLNG); runs at user protection level (TA_RNG3). */
    ctsk.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    if (prc != 0u)
    {
        ctsk.tskatr |= TA_ASSPRC; /* Honour the affinity below. */
        ctsk.assprc = prc;        /* Which core(s) it may run on. */
    }
#else
    (void)prc; /* Single-core build: no affinity. */
#endif

    /* tk_cre_tsk: create the task. It starts in the DORMANT state. */
    ID tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        app_check(tid, p_name);
        return tid;
    }

    /* tk_sta_tsk: DORMANT -> READY. The 0 is passed to the task as stacd. */
    app_check(tk_sta_tsk(tid, 0), p_name);
    return tid;
}

void
app_check (ER er, char const * p_what)
{
    if (er < E_OK)
    {
        tm_printf((UB *)"ERROR %s: %d\n", p_what, (INT)er);
    }
}

INT
usermain (void)
{
    T_RVER ver;

    /* tk_ref_ver: ask the kernel who it is. */
    (void)tk_ref_ver(&ver);
    tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
                    " (spec 0x%04x, product 0x%04x) ===\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    /* tk_get_prc() counts from 1. */
    tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1);
#else
    tm_printf((UB *)"single-core build\n");
#endif

    /* Phase 1: every subsystem creates its kernel objects. */
    app_check(obstacle_init(), "obstacle_init");
    app_check(motion_init(), "motion_init");
    app_check(line_follow_init(), "line_follow_init");
    app_check(terrain_init(), "terrain_init");
    app_check(communication_init(), "communication_init");

    /* Phase 2: every subsystem starts its tasks. */
    app_check(obstacle_start(), "obstacle_start");
    app_check(motion_start(), "motion_start");
    app_check(line_follow_start(), "line_follow_start");
    app_check(terrain_start(), "terrain_start");
    app_check(communication_start(), "communication_start");

    tm_printf((UB *)"all subsystems started\n\n");

#if APP_TEST
    /*
     * Test build (make TEST=1): run every subsystem's tests in turn, here in
     * the initial task, while the system is running. Tests may call
     * tk_dly_tsk() to let the other tasks work for a while.
     */
    tm_printf((UB *)"[TEST] === test run start ===\n");
    obstacle_test();
    motion_test();
    line_follow_test();
    terrain_test();
    communication_test();
    tm_printf((UB *)"[TEST] === test run end ===\n");
#endif

    /* Sleep forever. Returning from usermain() would shut down. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}

/* Private function definitions. */
