#ifndef __LED_H__
#define __LED_H__

#include <rtthread.h>

void rt_hw_led_init(void);
void rt_hw_led(rt_uint8_t led);

#endif
