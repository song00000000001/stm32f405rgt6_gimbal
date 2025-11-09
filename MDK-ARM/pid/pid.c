#include "pid.h"
#include "encoder.h"
#include "ble.h"
#include "motor.h"
#include <math.h>
#include "filter.h"

#define pid_deadzone 7
//#include "string.h"

volatile bool ANGLE_PID = false;

pid_pos pid_angle =   {.Kp = 1.0, .Ki = 0, .Kd = 0,.integral_max=2500,
	.output_max = 250,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0};//output_max要根据实测速度重新设置,不加低通174,加了就34
	
pid_inc pid_speed =   {.Kp = 0.0, .Ki = 20.0, .Kd = 0,
	.output_max = 20000,.target=0,.now=0,.output=0,.last_error=0};												
	
LowPassFilter myFilter={.alpha=0.1,.previous_output=0};


float pid_speed_task(int16_t speed,int16_t angle)
{
	// --- 1. 编码器绝对位置累加
	static int32_t absolute_position = 0;   
	static int32_t last_encoder_raw=0;
	static uint8_t pid_counter=0;
	int16_t current_encoder_raw = angle;
    int16_t encoder_delta = current_encoder_raw - last_encoder_raw;
	if (encoder_delta > 8192/2) encoder_delta -= 8192;
    else if (encoder_delta < -8192/2) encoder_delta += 8192;
    absolute_position += encoder_delta;
    last_encoder_raw = current_encoder_raw;
	// --- 2. 根据全局控制状态来执行PID逻辑 ---
	static ControlState_t last_control_state = CONTROL_DISABLED;

	 // 检查是否发生了从 "Disabled" 到 "Enabled" 的状态切换
    if (g_robot_control_state == CONTROL_DISABLED && last_control_state == CONTROL_DISABLED)
    {
        // 状态激活的瞬间！这是关键的重置点
        // 将当前电机的绝对位置，设定为新的目标位置
        pid_angle.target = (float)absolute_position * 360.0f / 8192.0f+140.0f;
        
        // 清零所有PID积分和历史误差，防止旧数据影响
        pid_angle.integral = 0;
        pid_angle.last_error = 0;
		pid_angle.last_now=0;
    
        pid_speed.last_error = 0;
    }

    // --- 3. 计算当前位置和速度 ---
    pid_angle.now = (float)absolute_position * 360.0f / 8192.0f+140.0f;
	#if 1
	pid_speed.now =	filterValue(&myFilter,speed);
	#else	
	pid_speed.now = speed;
    #endif
    // 更新上一次的状态
    last_control_state = g_robot_control_state;

	// --- 4. 只有在使能状态下才计算并返回PID输出 ---
    if (g_robot_control_state == CONTROL_ENABLED)
    {
        // 这里可以加入从遥控器更新 target 的逻辑
        // pid_angle.target += RC_CtrlData.remote.ch0 * 0.1f; // 例如：用摇杆控制目标角度
        pid_counter++;
        // 执行串级PID计算
        #if  pid_speed_mode
		#else
			if(pid_counter>=10){
				pid_speed.target=pid_cal_pos(&pid_angle);// 角度环的输出是速度环的目标
				pid_counter=0;
			}
		#endif
        return pid_cal_inc(&pid_speed);             // 速度环的输出是最终的电机电压/电流
    }
    else
    {
        // 如果是失能状态，返回0，不给电机任何力
        return 0;
    }
}

float pid_cal_pos(pid_pos *pid)
{
    float error = pid->target - pid->now;
	if(fabs(error)<0.2)
		return 0; 

    float derivative =  -(pid->now - pid->last_now);
	pid->last_now=pid->now;

	//计算输出
	pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

	//输出限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;
	
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
		

		if(fabs(pid_angle.now-pid_angle.target)<=0.10){
			pid->output=0;
		}
		
    return pid->output;
}

#if 0
	pid->integral += error;
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

#if 0	
	//死区补偿
	float dead_zone=5.4;
	if (pid->output > -dead_zone&&pid->output <0 &&fabs(error)>=0.2) 
			pid->output -=dead_zone;
    if (pid->output > 0&&pid->output <dead_zone &&fabs(error)>=0.2) 
			pid->output += dead_zone;
#endif	

#if 0	
		//死区补偿
		float dead_zone=5.4;
		if (pid->output > -dead_zone&&pid->output <0) 
			pid->output -=dead_zone;
		if (pid->output > 0&&pid->output <dead_zone) 
			pid->output += dead_zone;
#endif	