#include "pid.h"
#include "encoder.h"
#include "ble.h"
#include "motor.h"
#include <math.h>

#define pid_deadzone 7
//#include "string.h"

volatile bool ANGLE_PID = false;

pid_pos pid_angle =   {.Kp = 0.56, .Ki = 0.002, .Kd = 20.0,
                        .integral_max=50000,.output_max = 100,.target=30};

void pid_init(void)
{
	//HAL_TIM_Base_Start_IT(PID_TIM);
	ANGLE_PID=true;
}


float pid_cal_pos(pid_pos *pid)
{
    float error = pid->target - pid->now; 
#if 0
		pid->integral += error;
#else
	// 方案B：带死区的积分分离 (防止在0点附近抖动时积分乱跳)
	if(fabs(error) < 10.0) // 阈值依然是5.0
	{
			if(fabs(error) >= 0.2) // 在小误差区内部，再加一个积分死区
			{
					pid->integral += error;
			}
			else
				pid->integral = 0;
	}
	else
	{
			pid->integral = 0;//积分清零,防止饱和后意外无法归零导致陷阱点出现
	}
#endif

    //积分限幅
		if (pid->integral > pid->integral_max) {
				pid->integral = pid->integral_max;
		} else if (pid->integral < -pid->integral_max) {
				pid->integral = -pid->integral_max;
		} 

    float derivative =  -(pid->now - pid->last_now);
		pid->last_now=pid->now;
		
		/*尝试在误差低时给低kp抑制超调
		if(fabs(error)<5)
			pid->output = pid->Kp * error*0.1 + pid->Ki * pid->integral + pid->Kd * derivative;
		else
		*/
		//计算输出
		pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

		//输出限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;
	
		//死区补偿
	float dead_zone=5.4;
		if (pid->output > -dead_zone&&pid->output <0 &&fabs(error)>=0.2) 
			pid->output -=dead_zone;
    if (pid->output > 0&&pid->output <dead_zone &&fabs(error)>=0.2) 
			pid->output += dead_zone;
		
    return pid->output;
}


