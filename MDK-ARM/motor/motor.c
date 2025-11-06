#include "motor.h"

 /*
Motor M_1   = {.Tim = M1_TIM, .TIM_CHAN= M1_TIM_CHAN,
				 .IN1_port = M1_IN1_PORT, .IN1_pin = M1_IN1_PIN,
				 .IN2_port = M1_IN2_PORT, .IN2_pin = M1_IN2_PIN };

void Motor_init()
{
	Motor*motor=&M_1;
	HAL_GPIO_WritePin(motor->IN1_port, motor->IN1_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->IN2_port, motor->IN2_pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(motor->Tim,motor->TIM_CHAN);
}

void Motor_run(float speed)
{
	Motor*motor=&M_1;
	
	if(speed>100)
		speed=100;
	else if(speed<-100)
		speed=-100;
	
	if(speed > 0)
	{
		HAL_GPIO_WritePin(motor->IN1_port, motor->IN1_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->IN2_port, motor->IN2_pin, GPIO_PIN_RESET);
	}
	else if(speed < 0)
	{
		speed = -speed;
		HAL_GPIO_WritePin(motor->IN1_port, motor->IN1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->IN2_port, motor->IN2_pin, GPIO_PIN_SET);
	}
	else 
	{
		speed=0;
		HAL_GPIO_WritePin(motor->IN1_port, motor->IN1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->IN2_port, motor->IN2_pin, GPIO_PIN_RESET);
	}
	
	int16_t actual_speed = (int16_t)(speed * MOTOR_ARR * 0.01f);
	__HAL_TIM_SetCompare(motor->Tim, motor->TIM_CHAN, actual_speed); 

}

	 */
