#ifndef TEST_HARDWARE_GPIO_H
#define TEST_HARDWARE_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#define GPIO_IN 0
#define GPIO_OUT 1
#define GPIO_IRQ_EDGE_FALL 4u
#define GPIO_IRQ_EDGE_RISE 8u

void gpio_init (unsigned int pin);
void gpio_set_dir (unsigned int pin, bool out);
void gpio_put (unsigned int pin, bool value);
void gpio_add_raw_irq_handler (unsigned int pin, void (*handler)(void));
void gpio_set_irq_enabled (unsigned int pin, uint32_t events, bool enabled);
uint32_t gpio_get_irq_event_mask (unsigned int pin);
void gpio_acknowledge_irq (unsigned int pin, uint32_t events);

#endif
