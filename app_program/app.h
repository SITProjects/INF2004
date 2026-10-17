/**
 * @file app.h
 * @brief Definitions shared by the whole team. Changed only by team
 *        agreement.
 *
 * Anything that belongs to ONE subsystem goes in that subsystem's own header
 * instead (app_program/<name>/<name>.h).
 */

#ifndef APP_H
#define APP_H

/* Public includes. */
#include <tk/tkernel.h>  /* micro T-Kernel API (tk_xxx_yyy). */
#include <tm/tmonitor.h> /* T-Monitor API: tm_printf() console output. */
#include <bsp/libbsp.h>  /* Board helpers: gpio_set_pin/val, pwm_... */

/* Public types, constants, and macros. */

/*
 * Pins: free for projects are GP0..GP22 and GP26..GP28. GP0/GP1 are taken
 * by UART0 when built with "make CRASH_REPORT=1" (see README). Never use
 * GP23/24/25/29: they are wired to the Pico W radio. Claim every pin in
 * TEAM.md section 2 before using it.
 */

/*
 * Task priorities: 1 is the highest; the kernel's USB console (3) and
 * Wi-Fi (4) tasks sit above every application task. Each subsystem has its
 * own band (TEAM.md section 4), defined in its own header.
 */

/** Stack size of every application task, in bytes; tm_printf needs ~1 KB. */
#define TASK_STACK_SIZE 2048

/* Public function declarations. */

/**
 * @brief Create and start a task. Defined in app_main.c.
 * @param entry Function the task runs: void task (INT stacd, void * exinf).
 * @param pri Initial priority; 1 is the highest.
 * @param prc TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for any core.
 *        Ignored by the single-core build.
 * @param p_name Name printed if creating or starting the task fails.
 * @return The new task's ID, or a negative kernel error code.
 */
ID app_start_task (FP entry, PRI pri, UW prc, char const * p_name);

/**
 * @brief Print a readable message if a kernel call failed.
 * @param er Result of the call; negative values are kernel error codes.
 * @param p_what What was called, for the message.
 */
void app_check (ER er, char const * p_what);

/*
 * Every subsystem folder app_program/<name>/ provides these, in <name>.c
 * and <name>_test.c. usermain() calls ALL the _init() functions first and
 * only then ALL the _start() functions, so every kernel object exists
 * before any task can use it.
 *   <name>_init()   create this subsystem's kernel objects
 *   <name>_start()  create and start this subsystem's tasks
 *   <name>_test()   on-target tests, only in test builds (make TEST=1)
 */
ER obstacle_init (void);
ER motion_init (void);
ER line_follow_init (void);
ER terrain_init (void);
ER communication_init (void);

ER obstacle_start (void);
ER motion_start (void);
ER line_follow_start (void);
ER terrain_start (void);
ER communication_start (void);

#if APP_TEST
void obstacle_test (void);
void motion_test (void);
void line_follow_test (void);
void terrain_test (void);
void communication_test (void);
#endif

#endif /* APP_H */
