#include "bsp_can.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "ble.h"
#include "pid.h"

extern osMessageQId can_rx_queueHandle;

moto_info_t motor_info[MOTOR_MAX_NUM];
//数据接收数组
uint8_t can_rx_flag=0;
uint8_t can_read_flag=0;

void can1_rx(void const * argument){

  /* USER CODE BEGIN can1_rx */
  moto_info_t motor_info_0[MOTOR_MAX_NUM];
  TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 
  static uint16_t pid_counter=0;
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
  /* Infinite loop */
  for(;;)
  {

		// 1. 使用vTaskDelayUntil实现精准的周期性延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
		if(can_rx_flag){
			can_read_flag=1;     //互斥锁
			motor_info_0[0].rotor_angle=motor_info[0].rotor_angle;// motor_info_0[0].id=motor_info[0].id;// motor_info_0[0].torque_current=motor_info[0].torque_current;
			motor_info_0[0].rotor_speed=motor_info[0].rotor_speed;// motor_info_0[0].temp=motor_info[0].temp;// motor_info_0[0].dlc=motor_info[0].dlc; 
			can_read_flag=0;
      
			#if can_send_rx
				my_printf("id%d,dlc:%d,ang:%d,spe:%d,tem:%d,cur:%d\r\n",
				motor_info_0[0].id,motor_info_0[0].dlc,motor_info_0[0].rotor_angle,
				motor_info_0[0].rotor_speed,motor_info_0[0].temp,motor_info_0[0].torque_current);
			#endif

			
			pid_speed_task(motor_info_0[0].rotor_speed,motor_info_0[0].rotor_angle);
			pid_counter++;
			if(pid_counter>=10){
				pid_counter=0;
				pid_angle_task();
			}			

			//只有标志位激活才会驱动电机,且标准位任务优先级高于该任务,频率是100hz	
			if(sbus_receive_success){
				set_motor_voltage( 0,0,(int16_t)pid_speed.output,0,0);
			}
			
			#if can_send_pid
				#if pid_speed_mode	
					vofa_send(4,(float)pid_speed.target,(float)pid_speed.now,(float)(pid_speed.now - pid_speed.target),(float)pid_speed.output);
				#else
					 vofa_send(4,(float)pid_angle.target ,(float)pid_angle.now ,(float)(pid_angle.now - pid_angle.target) ,(float)pid_angle.output);
				#endif
			#endif	

			can_rx_flag=0;

			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
		}
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
		// HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
  }
  /* USER CODE END can1_rx */
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  static uint16_t can_cnt;
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];

  if (hcan->Instance == CAN1)
  {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
  }
  if ((rx_header.StdId >= FEEDBACK_ID_BASE)
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM)
   && !can_read_flag)                  // judge the can id
  {
    can_cnt ++;
    uint8_t index = 0;//rx_header.StdId - FEEDBACK_ID_BASE; // get motor index by can_id
    motor_info[index].id= rx_header.StdId - FEEDBACK_ID_BASE;  
    motor_info[index].dlc=rx_header.DLC;
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
    
    can_rx_flag=1;
  }
  if (can_cnt == 1000)
  {
    can_cnt = 0;
    LED_GREEN_TOGGLE(); // green led blink indicate can comunication successful 
  }
  
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
