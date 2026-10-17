/**
 * @file drv_ir_sensor.c
 * @brief Sequential ADC readings from two reflective IR line sensors,
 *        through the kernel's A/D converter device driver.
 */

#include "drv_ir_sensor.h"
#include <tk/tkernel.h>

/* Private types, constants, and macros. */
#define IR_SENSOR_ADC_DEVICE "adca" /* Kernel A/D converter, unit 0. */
#define IR_SENSOR_LEFT_CHANNEL 0    /* ADC0 on GP26. */
#define IR_SENSOR_RIGHT_CHANNEL 1   /* ADC1 on GP27. */
#define IR_SENSOR_ADC_MASK 0x0FFFu  /* 12-bit result. */

/* Static data. */
static ID adc_dd;
static bool b_adc_open = false;

/* Private function declarations. */
static bool read_channel (W channel, uint16_t * p_value);

/* Public function definitions. */

bool
ir_sensor_init (void)
{
    ID dd = tk_opn_dev((UB *)IR_SENSOR_ADC_DEVICE, TD_READ);
    if (dd < E_OK)
    {
        return false;
    }
    adc_dd = dd;
    b_adc_open = true;
    return true;
}

bool
ir_sensor_read (ir_sensor_reading_t * p_reading)
{
    if (!b_adc_open || (p_reading == NULL))
    {
        return false;
    }

    return read_channel(IR_SENSOR_LEFT_CHANNEL, &p_reading->left_raw) &&
           read_channel(IR_SENSOR_RIGHT_CHANNEL, &p_reading->right_raw);
}

/* Private function definitions. */

/* Select a channel, discard one conversion, keep the next. */
static bool
read_channel (W channel, uint16_t * p_value)
{
    UW raw = 0u;
    SZ asize = 0;

    for (INT i = 0; i < 2; i++)
    {
        ER er = tk_srea_dev(adc_dd, channel, &raw, 1, &asize);
        if ((er < E_OK) || (asize != 1))
        {
            return false;
        }
    }
    *p_value = (uint16_t)(raw & IR_SENSOR_ADC_MASK);
    return true;
}
