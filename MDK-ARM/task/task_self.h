#ifndef __TASK_SELF_H__
#define __TASK_SELF_H__

//Includes
#include "main.h"
#include "mpu6050.h"
#include "pid.h"
#include "ble.h"
//宏定义区

    //切换输出用
#define motor_id_global 4       	 //电机id选择,目前对pid计算和信息输出有效,因为还没有设计多电机控制
#define can_send_rx 1      				//电机读取信息输出

#define pid_speed_mode 1    		//1为速度环，0为位置环
#define pid_send 1         			//串级pid输出

#define mpu_send 1      		//mpu6050输出
#define mpu_send_angle_gyro 1   //mpu6050角度和陀螺仪输出

#define sbus_send_chan 1      //遥控器接收信号输出


    //debug用
#define ble_uart_send_debug 1       //初始化发送一些信息
#define ble_send_rx_buf_debug 0    //把串口1收到的发出来
#define sbus_send_rx_buf_debug 0     //把串口2收到的发出来
#define filter_enable 0
#define led_timer 50


//外部变量声明区
extern bool mpu_rx_flag;
extern bool sbus_rx_flag;
extern mpu6050_raw mpu_data_global;
extern uint8_t can_rx_flag;
extern pid_pos pid_angle_pitch;
extern pid_pos pid_speed_pitch;
extern pid_pos pid_angle_yaw;
extern pid_pos pid_speed_yaw;
extern volatile float led_freq;
extern volatile uint16_t g_led_brightness;
extern volatile ControlState_t g_robot_control_state;

//函数声明区
void debug_send_uart1(uint8_t t);

#endif
