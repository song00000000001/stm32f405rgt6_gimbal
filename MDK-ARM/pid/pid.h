#ifndef __PID_H__
#define __PID_H__

#include "main.h"
//#include "tim.h"
#include <stdbool.h>

typedef struct pid_pos
{
	float Kp,Ki,Kd;

	float target;
	float now;
	float last_now;

	float integral;
	float integral_max;

	float last_err;
	float output;
	float output_max;
}pid_pos;

//#define PID_TIM &htim1

extern pid_pos pid_angle;
extern volatile bool ANGLE_PID;

void pid_init(void);
float pid_cal_pos(pid_pos *pid);
void pid_task(void);
#endif
