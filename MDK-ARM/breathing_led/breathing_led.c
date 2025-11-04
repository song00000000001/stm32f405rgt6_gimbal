#include "breathing_led.h"

volatile uint16_t led_brightness=100;

void led_set_brightness(uint8_t x)//范围应该是1~5
{
	if(x>100)
		x=100;	
	led_brightness  =   x   ;
}


volatile uint16_t led_breath_freq=20;