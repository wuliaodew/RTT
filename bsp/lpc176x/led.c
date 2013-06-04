#include "LPC17xx.h"
#include "led.h"

void rt_hw_led_init(void)
{
    LPC_GPIO2->FIODIR0 = 0XFF; /* led0:P2.0 */
    LPC_GPIO0->FIODIR = 0x00200000; 
    LPC_GPIO0->FIOPIN |=  0x00200000; 
}

void rt_hw_led(rt_uint8_t led)
{         
    LPC_GPIO2->FIOPIN0 = led;
}


