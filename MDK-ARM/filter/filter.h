#ifndef __FILTER_H__
#define __FILTER_H__

#include "main.h"
// 定义一阶低通滤波器结构体
typedef struct {
    float alpha;           // 时间常数
    float previous_output; // 上一时刻的输出
} LowPassFilter;

void initializeFilter(LowPassFilter* filter, float alpha) ;
float filterValue(LowPassFilter* filter, float input) ;
	
#endif
