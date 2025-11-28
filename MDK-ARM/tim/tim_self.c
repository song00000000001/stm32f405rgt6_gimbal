#include "tim_self.h"
#include "breathing_led.h"

task_flag_str task_flags={
	.run_pid_inc_loop=false,
	.run_pid_pos_loop=false,
	.	change_target=false ,
	.led_brightness_adjust=false
};

volatile uint8_t pid_pos_counter,led_task_counter;
volatile uint16_t target_counter;

void tim5_init(){
	//pid_inc_counter=0;
	pid_pos_counter=0;
	led_task_counter=0;
	target_counter=0;
	HAL_TIM_Base_Start_IT(&htim5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 该函数在 stm32f1xx_hal_tim.c 中定义为弱函数(__weak)，由用户再定义
{
	//1kHz Timer ISR
	if(htim == &htim5)
	{

		// 任务1: 每1ms触发一次增量式pid(速度环) (1kHz)
		task_flags.run_pid_inc_loop = true;
		
		// 任务2: 每10ms运行一次位置式pid(角度环) (100Hz)
		pid_pos_counter++;
		if (pid_pos_counter >= 10) {
			task_flags.run_pid_pos_loop = true;
			pid_pos_counter = 0;
		}
		
		// 任务3:每1ms计算一次led的亮灭
		led_task_counter++;
		if (led_task_counter >= 20) {
			task_flags.led_brightness_adjust = true;
			led_task_counter = 0;
		}
		// 创建一个静态计数器，用于在0-99之间循环
		static uint16_t pwm_counter = 0;

		// 比较计数器和设定的亮度值
		if (pwm_counter < led_brightness) {
				// 计数值小于亮度值，输出低电平点亮LED (大部分开发板PC13是低电平点亮)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
		} else {
				// 计数值大于等于亮度值，输出高电平熄灭LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		}

		// 计数器自增
		pwm_counter++;

		// 如果计数器达到一个完整的PWM周期(100ms)，则清零
		// 这样就构成了一个周期为led_breath_freqms(1/led_breath_freq)的PWM波
		if (pwm_counter >= 10) {
				pwm_counter = 0;
		}
		
		// 任务4: 每2s改变一次target
		target_counter++;
		if(target_counter >= 1000){
			task_flags.change_target= true;
			target_counter=0;
		}
		
	}

}

/*
	
#	if 1
			vofa_send(2,(float)pid_angle.output,(float)pid_angle.now);	
#else
			// 使用sprintf直接格式化到字符串
				sprintf((char*)ble_txBuffer, "S%+06.2f%+06.2f%+06.2f%+06.2fE", 
				(float)pid_upright_t.now, 
				(float)pid_upright_t.target, 
				(float)pid_speed_t.now, 
				(float)pid_speed_t.target);

				// 发送BLE数据（假设tail是字符串长度，这里用strlen获取）
				BLE_print(ble_txBuffer, strlen((char*)ble_txBuffer));	
#endif
		
	}
}

*/

