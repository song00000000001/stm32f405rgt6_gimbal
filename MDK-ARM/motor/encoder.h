#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "tim.h"

void Encoder_Init();

#define EN1_TIM   &htim2
#define EN1_CHN   TIM_CHANNEL_1|TIM_CHANNEL_2

#endif