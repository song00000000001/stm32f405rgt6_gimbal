#ifndef BLE_H
#define BLE_H
#include "main.h"
#include "usart.h"
#include "dr16.h"

#define ble_uart &huart1
#define sbus_uart &huart2

#define ble_rx_buffer_size 14
#define SBUS_FRAME_SIZE 18
#define BLE_TX_BUF_LEN  32 

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


extern uint8_t ble_rx_buffer[];
extern uint8_t sbus_rx_buf[];

void ble_Init(void);
void ble_print(uint8_t* buf,uint16_t len);
void vofa_send(int num, ...);
void my_printf(const char *format, ...);
void ble_parse(uint8_t *buf);

#endif


