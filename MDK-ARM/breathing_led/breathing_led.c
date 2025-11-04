#include "breathing_led.h"

volatile uint16_t led_brightness = 0;

void led_set_brightness(uint8_t x)//范围应该是1~5
{
	if(x>100)
		x=100;	
	led_brightness  =   x   ;
}

volatile uint16_t led_breath_freq=20; 

uint8_t led_breath(uint8_t f){
	static uint8_t led_breath_flag =1;
	
	if(f==0){
		led_brightness=100;
		return 100;
	}
	
	//f=1hz,则需要延时1s=1000ms,
	//f_max=255,延时~=4ms
	uint8_t t=1000/f;

	if(led_brightness>99){
		led_brightness=-1;
	}
	else if(led_brightness==0)
	{
		led_brightness=1;
	}
	
	led_brightness+=led_breath_flag;
	
	return f;
}

