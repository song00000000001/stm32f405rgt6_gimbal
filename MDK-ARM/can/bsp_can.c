#include "bsp_can.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "ble.h"

extern osMessageQId can_rx_queueHandle;

CAN_TxHeaderTypeDef	TxHeader;      //发送
CAN_RxHeaderTypeDef	RxHeader;      //接收
 
uint8_t	RxData[8];  //数据接收数组，can的数据帧只有8帧

uint8_t Rx_Flag;
uint8_t Tx_Flag;
uint8_t RxBuf[8];
uint8_t TxBuf[8];
 
 uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len);
//moto_info_t motor_info[MOTOR_MAX_NUM];
//uint16_t can_cnt;

//init can filter, start can, enable can rx interrupt
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
  if (hcan->Instance == CAN1)
  {
#if 0  
  // 1. 从CAN硬件FIFO中获取消息
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_msg.header, rx_msg.data) == HAL_OK){
    {
        // 2. 将收到的消息打包后，通过队列发送给处理任务// 注意：CMSIS V1 osMessagePut传递的是指针
        osMessagePut(can_rx_queueHandle, (uint32_t)&rx_msg, 0);
    }
  }
#elif 0
	CAN_Message_t rx_msg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 直接发送结构体，FreeRTOS会自动拷贝内容
    xQueueSendFromISR(can_rx_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	  //my_printf((char*)received_msg.data);
#elif 1
		uint32_t i;
	
		Rx_Flag =1;  //接收标志位
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
		for(i=0;i<RxHeader.DLC;i++)  RxBuf[i]=RxData[i];  //用RxBuf转存RxData的数据
		my_printf((char*)RxBuf);
#else	  
static uint8_t can_cnt=0;	  
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  }
  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // judge the can id
  {
    can_cnt ++;
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
  }
  if (can_cnt == 500)
  {
    can_cnt = 0;
    LED_GREEN_TOGGLE(); // green led blink indicate can comunication successful 
  }
#endif  
  
  }

}

void can1_rx(void const * argument){

  /* USER CODE BEGIN can1_rx */
  CAN_Message_t received_msg; 
  /* Infinite loop */
  for(;;)
  {
    // 2. 阻塞等待，看是否收到了自己发的消息
   /* if (xQueueReceive(can_rx_queueHandle, &received_msg, pdMS_TO_TICKS(100)))
    {
      CAN_RxHeaderTypeDef h_t= received_msg.header;
      // 接收成功！
      // 在这里加断点，检查 received_msg.header.StdId 是否为 0x123
      // 以及 received_msg.data 是否为 "HELLOCAN"
      // 如果都正确，说明自收发测试成功！
      my_printf((char*)received_msg.data);
      LED_GREEN_TOGGLE(); // 接收成功就闪灯
    }    */

    osDelay(10);
  }
  /* USER CODE END can1_rx */
}

void can1_tx(void const * argument)
{
  /* USER CODE BEGIN can1_tx */
  /* Infinite loop */
    CAN_TxHeaderTypeDef tx_header;
    uint8_t    tx_data[8] = {'H', 'E', 'L', 'L', 'O', 'C', 'A', 'N'};
    uint32_t   tx_mailbox;

    // 配置要发送的报文头
    tx_header.StdId = 0x123;       // 测试用的ID
    tx_header.IDE   = CAN_ID_STD;  // 标准帧
    tx_header.RTR   = CAN_RTR_DATA; // 数据帧
    tx_header.DLC   = 8;           // 8字节数据长度

    for (;;)
    {
      // 1. 发送CAN报文
      if (!CAN_Send_Msg( tx_data, 8) )
      {
        // 发送成功，可以在这里点个灯或打印信息
		LED_GREEN_TOGGLE();
        my_printf("Message Sent Successfully!\r\n");
      }
      // 3. 任务延时，比如每500ms发送一次
      osDelay(500);
    }
  /* USER CODE END can1_tx */
}

#define CAN_RxExtId 0x1800D8D0
#define CAN_TxExtId 0x1800D0D8

/*CAN发送数据，入口参数为要发送的数组指针，数据长度，返回0代表发送数据无异常，返回1代表传输异常*/
uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len)
{	
  uint8_t i=0;
	uint32_t TxMailbox;
	uint8_t message[8];
 
	TxHeader.ExtId=CAN_TxExtId;        //扩展标识符(29位)
	TxHeader.IDE=CAN_ID_EXT;    //使用扩展帧
	TxHeader.RTR=CAN_RTR_DATA;  //数据帧
	TxHeader.DLC=len;    
	
  for(i=0;i<len;i++)
  {
		message[i]=msg[i];
	}
	
  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message, &TxMailbox) != HAL_OK)//发送
	{
		return 1;
	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
    return 0;
}

/**
  * @brief  send motor control message through can bus
  * @param  id_range to select can control id 0x1ff or 0x2ff
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */

 
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
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
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}

 #if 0 
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  }
  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // judge the can id
  {
    can_cnt ++;
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
  }
  if (can_cnt == 500)
  {
    can_cnt = 0;
    LED_GREEN_TOGGLE(); // green led blink indicate can comunication successful 
  }

  #endif