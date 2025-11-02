#ifndef BLE_H
#define BLE_H
#include "main.h"
#include "usart.h"

#define ble_uart &huart1

#define uart1_buf_size 14

extern uint8_t  uart1_buf[uart1_buf_size];//串口接收缓冲区
extern uint8_t  test_buf[];//串口发送测试字符串

void ble_Init(void);
void ble_print(uint8_t* buf,uint16_t len);
void vofa_send(int num, ...);
//struct pid_inc;  
//struct pid_pos;

//void BLE_ParsePID_pos(uint8_t *buf,struct pid_pos* pid);
//void BLE_ParsePID_inc(uint8_t *buf,struct pid_inc* pid);

#endif
