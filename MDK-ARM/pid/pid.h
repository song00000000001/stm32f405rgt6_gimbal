#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct pid_pos
{
	float Kp,Ki,Kd;

	float target;
	float now;
	float last_now;

	float integral;
	float integral_max;
    float integral_threshold;

	float last_error;
	float output;
	float output_max;
	
	float k_f;
}pid_pos;

typedef struct pid_inc
{
	float Kp,Ki,Kd;

	float target;
	float now;

	float last_error;
	float output;
	float output_max;

}pid_inc;

//#define PID_TIM &htim1

extern pid_pos pid_angle;
extern pid_inc pid_speed;

float pid_cal_pos_speed(pid_pos *pid);
float pid_cal_pos_angle(pid_pos *pid);
float pid_cal_inc(pid_inc *pid);
float pid_speed_task(float speed,int16_t angle,pid_pos *pid_angle,pid_pos *pid_speed,uint8_t motor_id);
float pid_cal_pos_angle_pitch(pid_pos *pid);
#endif
