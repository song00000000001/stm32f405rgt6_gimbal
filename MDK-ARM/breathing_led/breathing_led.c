#include "breathing_led.h"

uint16_t led_brightness=100;
uint16_t led_breath_freq=20;
void led_set_brightness(uint8_t x)//范围应该是1~5
{
	if(x>100)
		x=100;	
	else if(x<0)
		x=0;
	
	led_brightness  =   x   ;

}