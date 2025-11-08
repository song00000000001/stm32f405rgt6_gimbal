#ifndef BLE_H
#define BLE_H
#include "main.h"
#include "usart.h"
#include "stdbool.h"

//切换输出用
#define can_send_rx 0   //电机读取信息输出
#define can_send_pid 1  //电机串级pid输出
#define pid_speed_mode 0 
#define mpu_send_angle 0        //mpu6050角度输出
#define sbus_send_chan 0       //遥控器接收信号输出

//debug用
#define ble_uart_send_debug 0        //初始化发送一些信息
#define ble_send_rx_buf_debug 0   //把串口1收到的发出来
#define sbus_send_rx_buf_debug 0     //把串口2收到的发出来

#define ble_uart &huart1
#define huart_sbus &huart2

#define ble_rx_buffer_size 14
#define SBUS_FRAME_SIZE 25
#define BLE_TX_BUF_LEN  64 

#define led_timer 50
#define SBUS_CHANNEL_COUNT 16
	 
typedef struct {
    int16_t channels[SBUS_CHANNEL_COUNT];
    bool ch17;
    bool ch18;
    bool frame_lost;
    bool failsafe;
} SbusData_t;

extern bool sbus_receive_success;
extern float led_freq;
extern bool sbus_rx_flag;
extern SbusData_t my_sbus_data;



void ble_Init(void);
void ble_print(uint8_t* buf,uint16_t len);
void vofa_send(int num, ...);
void my_printf(const char *format, ...);

#endif

