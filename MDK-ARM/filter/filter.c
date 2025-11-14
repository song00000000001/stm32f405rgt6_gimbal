#include "filter.h"

 /*
    例子：低通滤波器使用
    // 初始化滤波器，设置时间常数为0.2
    float alpha = 0.2;
    LowPassFilter myFilter;
    initializeFilter(&myFilter, alpha);
		filterValue(&myFilter,input);
*/
// 初始化滤波器
void initializeFilter(LowPassFilter* filter, float alpha) {
    filter->alpha = alpha;
    filter->previous_output = 0.0;
}
 
// 一阶低通滤波函数
float filterValue(LowPassFilter* filter, float input) {
    // 计算输出
    float output = (1.0 - filter->alpha) * filter->previous_output + filter->alpha * input;
 
    // 更新上一次的输出
    filter->previous_output = output;
 
    return output;
}

//互补滤波器,200hzmpu,1khz motor
void Filter_Init(ComplementaryFilter* f, float cutoff_freq) {
  f->tau = 1.0f / (2 * M_PI * cutoff_freq);
  f->alpha = f->tau / (f->tau + 0.005f); // 默认200Hz对应dt=5ms
  f->last_omega = 0;
}

// 在1kHz电机中断中调用
float Filter_UpdateMotor(ComplementaryFilter* f, float motor_omega) {
  f->last_omega = (1 - f->alpha) * motor_omega + f->alpha * f->last_omega;
  return f->last_omega;
}

// 在200Hz MPU中断中调用
float Filter_UpdateMPU(ComplementaryFilter* f, float mpu_omega) {
  f->last_omega = f->alpha * mpu_omega + (1 - f->alpha) * f->last_omega;
  return f->last_omega;
}



