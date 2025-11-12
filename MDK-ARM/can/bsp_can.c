#include "bsp_can.h"
#include "task_self.h"
moto_info_t motor_info[MOTOR_MAX_NUM];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static uint16_t can_cnt;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];
	
    if (hcan->Instance == CAN1)
    {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

        if ((rx_header.StdId == (FEEDBACK_ID_BASE+1))) //6020,id1,pitch,205+1
        {
			can_cnt ++;
			motor_info[0].motor_angle    = ((rx_data[0] << 8) | rx_data[1]);//motor_info[0].id= rx_header.StdId - FEEDBACK_ID_BASE; //motor_info[0].torque_current = ((rx_data[4] << 8) | rx_data[5]); 
			motor_info[0].motor_speed    = ((rx_data[2] << 8) | rx_data[3]);//motor_info[0].dlc=rx_header.DLC;//motor_info[0].temp           =   rx_data[6];
            can_rx_flag=1;
        }
        if(rx_header.StdId <= (CAN_3510Moto_ID+3) 
        && rx_header.StdId>=(CAN_3510Moto_ID+1)
        )
        {
        can_cnt ++;
        uint8_t index = rx_header.StdId-CAN_3510Moto_ID;//rx_header.StdId - FEEDBACK_ID_BASE; // get motor index by can_id
        motor_info[index].motor_angle    = ((rx_data[0] << 8) | rx_data[1]);
        motor_info[index].motor_speed    = ((rx_data[2] << 8) | rx_data[3]);
        }
    }
  
    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
        if ((rx_header.StdId == (FEEDBACK_ID_BASE+3))) //6020,id4,yaw  ,    208
        {
            can_cnt ++;
            motor_info[4].motor_angle    = ((rx_data[0] << 8) | rx_data[1]);//motor_info[0].id= rx_header.StdId - FEEDBACK_ID_BASE; //motor_info[0].torque_current = ((rx_data[4] << 8) | rx_data[5]); 
            motor_info[4].motor_speed    = ((rx_data[2] << 8) | rx_data[3]);//motor_info[0].dlc=rx_header.DLC;//motor_info[0].temp           =   rx_data[6];
            can_rx_flag=1;
        }
    }
    
    if (can_cnt == 1000)
    {
        can_cnt = 0;
        //LED_GREEN_TOGGLE(); // green led blink indicate can comunication successful 
    }
  
}

_Bool set_motor_voltage(int16_t StdId, int16_t v1, int16_t v2, int16_t v3, int16_t v4,CAN_HandleTypeDef *hcan)
{

  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = StdId;

  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  if(HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)//发送
	{
		return 1;
	}
  else 
    return 0;
}

  void can1_filter_init(void)
{
    CAN_FilterTypeDef can_filter;

    can_filter.FilterBank = 0; // 为CAN1使用过滤器组0
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0;
    can_filter.FilterIdLow = 0;
    can_filter.FilterMaskIdHigh = 0;
    can_filter.FilterMaskIdLow = 0;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14; // 关键：定义CAN2的起始过滤器组

    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void can2_filter_init(void)
{
    CAN_FilterTypeDef can_filter;

    // 关键：为CAN2使用一个在它管辖范围内的过滤器组，例如第14组
    can_filter.FilterBank = 14; 
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0;
    can_filter.FilterIdLow = 0;
    can_filter.FilterMaskIdHigh = 0;
    can_filter.FilterMaskIdLow = 0;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    // can_filter.SlaveStartFilterBank 在为CAN2配置时是无效的，可以不设置

    HAL_CAN_ConfigFilter(&hcan2, &can_filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


