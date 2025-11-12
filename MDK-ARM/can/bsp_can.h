#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"

#define CAN_3510Moto_ID  0x200
#define FEEDBACK_ID_BASE      0x205
#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         5
#define LED_GREEN_TOGGLE()  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14)
#define LED_GREEN_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET)
#define LED_GREEN_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET)
																	  
#define CAN_RxExtId 0x1800D8D0
#define CAN_TxExtId 0x1800D0D8

typedef struct
{
    uint8_t id;  
    //uint8_t  dlc;// 数据长度码  
    //uint8_t  temp;  
    int16_t motor_speed;
    int16_t  motor_angle;
    //int16_t  torque_current;
}moto_info_t;//3+2*3=9

extern moto_info_t motor_info[MOTOR_MAX_NUM];

void can1_filter_init(void);
void can2_filter_init(void);
_Bool set_motor_voltage(int16_t StdId, int16_t v1, int16_t v2, int16_t v3, int16_t v4,CAN_HandleTypeDef *hcan);

#endif
