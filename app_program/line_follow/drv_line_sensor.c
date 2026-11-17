/**
 * @file drv_line_sensor.c
 * @brief Digital DO inputs of the two line IR modules (GP26, GP27).
 *
 * The pads are written directly, as the encoder driver does, because
 * gpio_set_pin() leaves the pulls at their reset values.
 */

#include "drv_line_sensor.h"
#include <stdint.h>
#include "../app.h"
#include "line_follow_config.h"

/* Private types, constants, and macros. */

/* Private function declarations. */
static void configure_pin (uint32_t pin);
static bool pin_black (uint32_t pin);

/* Public function definitions. */

void
drv_line_sensor_init (void)
{
    configure_pin(LINE_LEFT_PIN);
    configure_pin(LINE_RIGHT_PIN);
}

line_sensor_reading_t
drv_line_sensor_read (void)
{
    line_sensor_reading_t const reading = {
        .b_left_black = pin_black(LINE_LEFT_PIN),
        .b_right_black = pin_black(LINE_RIGHT_PIN)};
    return reading;
}

/* Private function definitions. */

/* Input with pull-down and Schmitt trigger, read through SIO. The
 * modules pull DO up themselves; unplugged, it reads 0. */
static void
configure_pin (uint32_t pin)
{
    out_w(GPIO_OE_CLR, 1u << pin);
    out_w(GPIO(pin), GPIO_IE | GPIO_PDE | GPIO_SHEMITT | GPIO_OD);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
}

static bool
pin_black (uint32_t pin)
{
    return gpio_get_val(pin) == LINE_DO_BLACK_LEVEL;
}
