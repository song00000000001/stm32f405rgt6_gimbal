#ifndef __FILTER_H__
#define __FILTER_H__

#include "main.h"
#define M_PI 3.14159265358979323846

// 定义一阶低通滤波器结构体
typedef struct {
    float alpha;           // 时间常数
    float previous_output; // 上一时刻的输出
} LowPassFilter;

typedef struct {
  float alpha;       // 融合系数(0~1)
  float last_omega;  // 上一次滤波输出
  float tau;         // 时间常数
} ComplementaryFilter;

void initializeFilter(LowPassFilter* filter, float alpha) ;
float filterValue(LowPassFilter* filter, float input) ;
void Filter_Init(ComplementaryFilter* f, float cutoff_freq) ;
float Filter_UpdateMPU(ComplementaryFilter* f, float mpu_omega) ;
float Filter_UpdateMotor(ComplementaryFilter* f, float motor_omega) ;	
#endif
