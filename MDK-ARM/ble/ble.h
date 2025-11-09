#ifndef BLE_H
#define BLE_H
#include "main.h"
#include "usart.h"
#include "stdbool.h"
#include "dr16.h"
			   
//切换输出用
#define can_send_rx 1           //电机读取信息输出
#define can_send_pid 1         //电机串级pid输出
#define pid_speed_mode 1 
#define mpu_send_angle 0      //mpu6050角度输出
#define sbus_send_chan 1      //遥控器接收信号输出
#define motor_id 4

//debug用
#define ble_uart_send_debug 1       //初始化发送一些信息
#define ble_send_rx_buf_debug 0    //把串口1收到的发出来
#define sbus_send_rx_buf_debug 0     //把串口2收到的发出来

#define ble_uart &huart1
#define huart_sbus &huart2

#define ble_rx_buffer_size 14
#define SBUS_FRAME_SIZE 18
#define BLE_TX_BUF_LEN  64 

#define led_timer 50
#define SBUS_CHANNEL_COUNT 16
	 
typedef struct {
     /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
  bool frame_lost;
  bool failsafe;
} SbusData_t;

typedef enum {
  CONTROL_DISABLED,
  CONTROL_ENABLED
} ControlState_t;

extern uint8_t sbus_receive_success;
extern float led_freq;
extern bool sbus_rx_flag;
extern SbusData_t my_sbus_data;
extern bool sbus_read_fine_flag;
extern volatile ControlState_t g_robot_control_state;


void ble_Init(void);
void ble_print(uint8_t* buf,uint16_t len);
void vofa_send(int num, ...);
void my_printf(const char *format, ...);

#endif


