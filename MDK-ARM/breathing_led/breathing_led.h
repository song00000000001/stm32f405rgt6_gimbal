#ifndef breathing_led_h
#define breathing_led_h
#include "main.h"

extern volatile uint16_t led_brightness;
extern volatile uint16_t led_breath_freq;

void led_set_brightness(uint8_t x);

#endif
