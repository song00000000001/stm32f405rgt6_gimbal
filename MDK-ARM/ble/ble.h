#ifndef BLE_H
#define BLE_H
#include "main.h"
#include "usart.h"
#include "stdbool.h"

#define can_send_pid 0
#define ble_send_rx_buf_debug 0
#define sbus_send_rx_buf_debug 1
#define mpu_send_angle 0
#define sbus_send_chan 1
#define ble_uart_send_debug 1

#define ble_uart &huart1
#define huart_sbus &huart2

#define ble_rx_buffer_size 14
#define SBUS_FRAME_SIZE 25
#define BLE_TX_BUF_LEN  256 

#define led_timer 50
#define SBUS_CHANNEL_COUNT 16

extern bool sbus_receive_success;
extern float led_freq;

typedef struct {
    int16_t channels[SBUS_CHANNEL_COUNT];
    bool ch17;
    bool ch18;
    bool frame_lost;
    bool failsafe;
} SbusData_t;

void ble_Init(void);
void ble_print(uint8_t* buf,uint16_t len);
void vofa_send(int num, ...);
void my_printf(const char *format, ...);

#endif

