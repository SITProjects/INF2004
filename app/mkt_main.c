#include <tk/tkernel.h>
#include <tm/tmonitor.h>

EXPORT INT
usermain (void)
{
    while (1)
    {
        tm_printf((UB *)"INF2004 running under microkernel\n");
        tk_dly_tsk(1000);
    }
}
