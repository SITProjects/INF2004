/* Host-only doubles: time advances solely through requested kernel delays. */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "app.h"
#include "obstacle/obstacle_calibration.h"
#include "obstacle/drv_ultrasonic.h"
static jmp_buf finish;
static unsigned long now, last_trigger, retry_time, pending_time;
static int count, limit, retry, pending, failure;
int
servo_init (int pin)
{
    assert(pin == 12);
    puts("INIT,SERVO");
    return failure == 1;
}
int
ultrasonic_init (int trig, int echo)
{
    assert(trig == 16 && echo == 17);
    puts("INIT,ULTRASONIC");
    return failure == 2;
}
void
servo_set_angle (int angle)
{
    printf("SERVO,%lu,%d\n", now, angle);
}
int
tk_get_otm (SYSTIM * t)
{
    t->lo = now;
    return 0;
}
int
tk_dly_tsk (RELTIM delay)
{
    assert(delay > 0 && delay % 10 == 0);
    now += delay;
    return 0;
}
int
tk_slp_tsk (int timeout)
{
    (void)timeout;
    longjmp(finish, 1);
}
void
tk_ext_tsk (void)
{
    puts("EXIT");
    longjmp(finish, 1);
}
int
tm_printf (UB * format, ...)
{
    va_list ap;
    va_start(ap, format);
    int n = vprintf((char *)format, ap);
    va_end(ap);
    return n;
}
bool
ultrasonic_trigger (void)
{
    if (count >= limit)
    {
        longjmp(finish, 1);
    }
    if (!retry)
    {
        retry = 1;
        retry_time = now;
        return false;
    }
    assert(now >= retry_time + 10);
    if (count && now < last_trigger + 60)
    {
        retry_time = now;
        return false;
    }
    retry = 0;
    pending = 0;
    last_trigger = now;
    ++count;
    printf("TRIGGER,%lu\n", now);
    return true;
}
ultrasonic_result_t
ultrasonic_read (uint16_t * echo)
{
    if (!pending)
    {
        pending = 1;
        pending_time = now;
        *echo = 777;
        return ULTRASONIC_PENDING;
    }
    assert(now >= pending_time + 10);
    *echo = 1749;
    return (ultrasonic_result_t)(1 + ((count - 1) % 3));
}
int
main (int argc, char ** argv)
{
    assert(argc == 3);
    limit = atoi(argv[1]);
    failure = atoi(argv[2]);
    if (!setjmp(finish))
    {
        obstacle_calibration_run(16, 17, 12);
    }
    return 0;
}
