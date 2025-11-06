#include "bsp_can.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "ble.h"

extern osMessageQId can_rx_queueHandle;

//数据接收数组
uint16_t can_cnt;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  moto_info_t motor_info[MOTOR_MAX_NUM];

  if (hcan->Instance == CAN1)
  {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
		//my_printf("Message received Successfully!\r\n");
  }
  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // judge the can id
  {
    can_cnt ++;
    uint8_t index = 0;//rx_header.StdId - FEEDBACK_ID_BASE; // get motor index by can_id
    motor_info[index].id= rx_header.StdId - FEEDBACK_ID_BASE;  
    motor_info[index].dlc=rx_header.DLC;
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 将消息发送到队列。使用FromISR版本
    xQueueSendFromISR(can_rx_queueHandle, &motor_info[0], &xHigherPriorityTaskWoken);
    // 如果发送操作唤醒了一个更高优先级的任务，则在中断退出时进行一次任务切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  }
  if (can_cnt == 1000)
  {
    can_cnt = 0;
    LED_GREEN_TOGGLE(); // green led blink indicate can comunication successful 
  }
  
}

void can1_rx(void const * argument){

  /* USER CODE BEGIN can1_rx */
  moto_info_t local_motor_info[MOTOR_MAX_NUM];
  /* Infinite loop */
  for(;;)
  {
    // 阻塞等待，直到can_rx_queueHandle中有数据
    if(xQueueReceive(can_rx_queueHandle,local_motor_info,portMAX_DELAY)){
    
      #if 1
      vofa_send(2,(float)local_motor_info[0].rotor_angle,(float)local_motor_info[0].rotor_speed);
	  //vofa_send(3,(float)local_motor_info[0].rotor_angle,(float)local_motor_info[0].rotor_speed,(float)local_motor_info[0].torque_current);
      #else
      my_printf("id%d,dlc:%d,ang:%d,spe:%d,tem:%d,cur:%d\r\n",
		    local_motor_info[0].id          ,local_motor_info[0].dlc,
        local_motor_info[0].rotor_angle ,local_motor_info[0].rotor_speed,
        local_motor_info[0].temp        ,local_motor_info[0].torque_current);
      #endif
    }
    osDelay(10);
  }
  /* USER CODE END can1_rx */
}

void can1_tx(void const * argument)
{
    for (;;)
    {
      // 发送CAN报文
	  set_motor_voltage( 0, 1000,0,0,0);
      // 任务延时，比如每500ms发送一次
      osDelay(10);
    }
  /* USER CODE END can1_tx */
}
 
_Bool set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{

  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x1ff):(0x2ff);
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
  if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)//发送
	{
		return 1;
	}
  else 
    return 0;

  //HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);   
}

  
void can_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
  HAL_CAN_Start(&hcan1);                          // start can1
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can1 rx interrupt
}

void CAN_Filter_Init_AcceptAll(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 仍然使用屏蔽位模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位或16位都可以

    // ****** 关键点 ******
    // 因为Mask全为0，所以ID和RTR/IDE位是什么都无所谓了，直接设为0即可
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    
    // ****** 关键点 ******
    // 将屏蔽位全部设置为0，实现“全通”
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    // 后续的 HAL_CAN_Start 和 HAL_CAN_ActivateNotification 保持不变
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}