#ifndef __MOTOR_H__
#define __MOTOR_H__
	  /*
#include "main.h"
#include "tim.h"

#define M1_TIM	&htim1
#define M1_TIM_CHAN 	TIM_CHANNEL_1
#define M1_IN1_PORT AIN1_GPIO_Port
#define M1_IN1_PIN  AIN1_Pin
#define M1_IN2_PORT AIN2_GPIO_Port
#define M1_IN2_PIN  AIN2_Pin

#define MOTOR_ARR 7199

typedef struct Motor
{
	TIM_HandleTypeDef* Tim;
    uint16_t TIM_CHAN;
    
	GPIO_TypeDef *IN1_port;
    uint16_t IN1_pin;
	
	GPIO_TypeDef *IN2_port;
    uint16_t IN2_pin;
	
}Motor;

extern Motor M_1;

void Motor_init();

void Motor_run(float speed);
*/
#endif
