#include "pid.h"
#include "encoder.h"
#include "ble.h"
#include "motor.h"
#include <math.h>
#include "filter.h"

#define pid_deadzone 7
//#include "string.h"

volatile bool ANGLE_PID = false;

pid_pos pid_angle =   {.Kp = 5.62, .Ki = 0, .Kd = 0,.integral_max=250000,
	.output_max = 68,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0};//output_max要根据实测速度重新设置,不加低通174,加了就34
	
pid_inc pid_speed =   {.Kp = 0.0, .Ki = 20.0, .Kd = 0,
	.output_max = 20000,.target=0,.now=0,.output=0,.last_error=0};												
	
LowPassFilter myFilter={.alpha=0.01,.previous_output=0};
  

static int32_t absolute_position = 0;   
static int32_t last_encoder_raw=0;

float pid_speed_task(int16_t speed,int16_t angle)
{
	int16_t current_encoder_raw = angle;
    int16_t encoder_delta = current_encoder_raw - last_encoder_raw;
		//如果电机在一个采样周期内转动超过了半圈（8192/4个计数），这个算法会失效。
    if (encoder_delta > 8192/4) // 用范围的一半作为阈值最稳妥
    {
        encoder_delta -= 8192; // 从 (2^16) 中减去，得到真实的负向增量
    }
    else if (encoder_delta < -8192/4)
    {
        encoder_delta += 8192; // 加上 (2^16)，得到真实的正向增量
    }
    absolute_position += encoder_delta;
    last_encoder_raw = current_encoder_raw;
	pid_angle.now = absolute_position*360/8192.0f;
#if 0
	pid_speed.now =	filterValue(&myFilter,speed);
#else	
	pid_speed.now = speed;
#endif
	return pid_cal_inc(&pid_speed);
}

void pid_angle_task()
{
	#if  pid_speed_mode
	#else
		pid_speed.target=pid_cal_pos(&pid_angle);
	#endif
}

float pid_cal_pos(pid_pos *pid)
{
    float error = pid->target - pid->now;
	if(fabs(error)<0.25)
		return 0; 
#if 0
		pid->integral += error;
#elif 0
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
	//积分限幅
	if (pid->integral > pid->integral_max) {
			pid->integral = pid->integral_max;
	} else if (pid->integral < -pid->integral_max) {
			pid->integral = -pid->integral_max;
	} 
#endif

    float derivative =  -(pid->now - pid->last_now);
	pid->last_now=pid->now;

	//计算输出
	pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

	//输出限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;


#if 0	
	//死区补偿
	float dead_zone=5.4;
	if (pid->output > -dead_zone&&pid->output <0 &&fabs(error)>=0.2) 
			pid->output -=dead_zone;
    if (pid->output > 0&&pid->output <dead_zone &&fabs(error)>=0.2) 
			pid->output += dead_zone;
#endif		
    return pid->output;
}


float pid_cal_inc(pid_inc *pid)
{
    // 1. 计算当前误差
    float error = pid->target - pid->now;

    // 2. 计算增量 ΔOutput
    // ΔOutput = Kp * [e(k) - e(k-1)] + Ki * e(k)
    float delta_output = pid->Kp * (error - pid->last_error) + pid->Ki * error;

    // 3. 更新历史误差，为下一次计算做准备
    pid->last_error = error;

    // 4. 累积增量得到当前理论输出
    // Output(k) = Output(k-1) + ΔOutput
    pid->output += delta_output;

    // 5. 处理输出饱和（抗积分饱和）
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    }
    else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }
		
#if 0	
		//死区补偿
		float dead_zone=5.4;
		if (pid->output > -dead_zone&&pid->output <0) 
			pid->output -=dead_zone;
		if (pid->output > 0&&pid->output <dead_zone) 
			pid->output += dead_zone;
#endif	
		if(fabs(pid_angle.now-pid_angle.target)<=0.10){
			pid->output=0;
		}
		
    return pid->output;
}
