#ifndef __TIM_SELF_H__
#define __TIM_SELF_H__

#include "main.h"
#include "tim.h"
#include <stdbool.h>

typedef struct task_flag_str{
	bool run_pid_inc_loop;
	bool run_pid_pos_loop;
	bool led_brightness_adjust;
	bool change_target;

}task_flag_str;

extern task_flag_str task_flags;

void tim5_init();
#endif
