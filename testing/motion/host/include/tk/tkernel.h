/* Just enough of the kernel API for host tests of motion_command.c. */
#ifndef TK_TKERNEL_H
#define TK_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int32_t ER;
typedef int32_t ID;
typedef int32_t INT;
typedef uint32_t UINT;
typedef int32_t W;
typedef uint32_t UW;
typedef uint8_t UB;
typedef int32_t PRI;
typedef void (*FP) (void);
typedef struct
{
    W hi;
    UW lo;
} SYSTIM;

#define E_OK 0
#define E_PAR (-17)
#define E_OBJ (-41)

ER tk_get_otm (SYSTIM * p_time);

#endif /* TK_TKERNEL_H */
