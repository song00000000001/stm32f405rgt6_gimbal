#include "pid.h"

#include <math.h>
#include "filter.h"
#include "task_self.h"
#include "bsp_can.h"

LowPassFilter myFilter={.alpha=0.1,.previous_output=0};

int16_t g_compensation=4900;


//--- yaw and pitch												 
pid_pos pid_angle_pitch =   {.Kp = 11, .Ki = 0.03, .Kd = 0,.integral_max=200,
	.output_max = 150,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=2};
pid_pos pid_speed_pitch =   {.Kp = 200, .Ki = 0, .Kd = 0,.integral_max=0,.k_f =110,
	.output_max = 25000,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=20};									
pid_pos pid_angle_yaw =   {.Kp = 14.5, .Ki = 0.1, .Kd = 40,.integral_max=600,
	.output_max = 600,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=1};
pid_pos pid_speed_yaw =   {.Kp = 600, .Ki = 0, .Kd = 0,.integral_max=25000, .k_f=30,
	.output_max = 25000,.target=0,.now=0,.last_now=0,.integral=0,.output=0,.last_error=0,.integral_threshold=20};

float pid_speed_angle_task(float speed,int16_t angle,pid_pos *pid_angle,pid_pos *pid_speed,uint8_t motor_id)
{
    // --- 0. 静态变量定义 ---
    // --- 1. 编码器绝对位置累加
    static int32_t absolute_position[5] = {0};   
    static int32_t last_encoder_raw[5]={0};
	if(pid_speed==NULL || pid_angle==NULL)
        return 0;
    // 更新绝对位置,基于电机的角度反馈
    int16_t current_encoder_raw = angle;
    int16_t encoder_delta = current_encoder_raw - last_encoder_raw[motor_id];
    if (encoder_delta > 8192/2) encoder_delta -= 8192;
    else if (encoder_delta < -8192/2) encoder_delta += 8192;
    absolute_position[motor_id] += encoder_delta;
    last_encoder_raw[motor_id] = current_encoder_raw;

    // --- 2. 根据全局控制状态来执行PID逻辑 ---
    // 在失能状态下，重置PID状态
    if (g_robot_control_state == CONTROL_DISABLED)
    {
        // 将当前电机的绝对位置，设定为新的目标位置
        pid_angle->target =pid_angle->now;
        
        // 清零所有PID积分和历史误差，防止旧数据影响
        pid_angle->integral = 0;
        pid_angle->last_error = 0;
        pid_angle->last_now=0;
        pid_speed->integral = 0;
        pid_speed->last_error = 0;
        pid_speed->last_now=0;
    }

    // --- 3. 计算当前位置和速度 ---
    pid_angle->now = (float)absolute_position[motor_id] * 360.0f / 8192.0f+140.0f;
	
	#if filter_enable
		pid_speed->now =	filterValue(&myFilter,speed);
	#else	
		pid_speed->now = speed;
    #endif

	// --- 4. 只有在使能状态下才计算并返回PID输出 ---
    if (g_robot_control_state == CONTROL_ENABLED)
    {
        // 执行串级PID计算
        #if  pid_speed_mode
		#else
            if(motor_id==4)
                pid_speed->target=pid_cal_pos_angle(pid_angle);// 角度环的输出是速度环的目标
            else if(motor_id==0)
                pid_speed->target=-pid_cal_pos_angle_pitch(pid_angle);
			else{
				 motor_id=motor_id;
			}
		#endif
        switch(motor_id){
            case 4:
                return pid_cal_pos_speed(pid_speed);
            case 0:
                return (pid_cal_pos_speed(pid_speed)+g_compensation);
            default:
                return 0;
        }
    }
    else
    {
        // 如果是失能状态，返回0，不给电机任何力
        return 0;
    }
}

float pid_speed_task(float speed,pid_pos *pid_speed,uint8_t motor_id)
{
	if(pid_speed==NULL)
        return 0;
    // --- 3. 计算当前速度 ---
	#if filter_enable
		pid_speed->now =	filterValue(&myFilter,speed);
	#else	
		pid_speed->now = speed;
    #endif

	// --- 4. 只有在使能状态下才计算并返回PID输出 ---
    if (g_robot_control_state == CONTROL_ENABLED)
    {
        switch(motor_id){
            case 1:
                return pid_cal_pos_speed(pid_speed);
            case 2:
                return pid_cal_pos_speed(pid_speed);
            case 3:
                return pid_cal_pos_speed(pid_speed);
            default:
                return 0;
        }
    }
    else
    {
        pid_speed->target =0;
        pid_speed->integral = 0;
        pid_speed->last_error = 0;
        pid_speed->last_now=0;
        return 0;
    }
}

float pid_cal_pos_speed(pid_pos *pid)
{
    float error = pid->target - pid->now;

	//带死区的积分分离 (防止在0点附近抖动时积分乱跳)
	if(fabs(error) < pid->integral_threshold) // 阈值依然是5.0
	{
        if(fabs(error) >= 0.1) // 在小误差区内部，再加一个积分死区
        {
            pid->integral += error;
        }
        else{
            pid->integral = 0;
            pid->output=0;
            return 0; 
        }
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

    float derivative =  -(pid->now - pid->last_now);
	pid->last_now=pid->now;

	//计算输出
	pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative+pid->k_f*pid->target;

	//输出限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;
	
    return pid->output;
}


float pid_cal_pos_angle(pid_pos *pid)
{
    float error = pid->target - pid->now;

	//带死区的积分分离 (防止在0点附近抖动时积分乱跳)
	if(fabs(error) < pid->integral_threshold) // 阈值依然是5.0
	{
        if(fabs(error) >= 0.1) // 在小误差区内部，再加一个积分死区
        {
            pid->integral += error;
        }
        else{
            pid->integral = 0;
            pid->output=0;
            return 0; 
        }
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

    float derivative =  -(pid->now - pid->last_now);
	pid->last_now=pid->now;

	//计算输出
	pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

	//输出限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;
	
    return pid->output;
}
 
float pid_cal_pos_angle_pitch(pid_pos *pid)
{
    float error = pid->target - pid->now;

	//带死区的积分分离 (防止在0点附近抖动时积分乱跳)
	if(fabs(error) < pid->integral_threshold) // 阈值依然是5.0
	{
        if(fabs(error) >= 0.1) // 在小误差区内部，再加一个积分死区
        {
            pid->integral += error;
        }
        else{
            return pid->Ki*pid->integral; 
        }
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