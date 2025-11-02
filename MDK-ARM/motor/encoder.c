#include "encoder.h"

void Encoder_Init()
{
	HAL_TIM_Encoder_Start(EN1_TIM, EN1_CHN);
}