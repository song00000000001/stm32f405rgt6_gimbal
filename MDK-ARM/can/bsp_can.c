#include "bsp_can.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "ble.h"
#include "pid.h"
#include "string.h"
#include "mpu6050.h"

extern osMessageQId can_rx_queueHandle;
extern bool mpu_data_busy,mpu_rx_flag;
extern mpu6050_data_t mpu_data_global;
moto_info_t motor_info[MOTOR_MAX_NUM];
/*
0:6020,id2,0x205+1,set_motor_voltage(0x1FF,0, motor_output[0], 0, 0, &hcan1);
1:3508,id1,0x200+1,set_motor_voltage( 0x200,motor_output[0],0,0,0,&hcan1);
2:3508,id2,0x200+2,set_motor_voltage( 0x200,0,motor_output[0],0,0,&hcan1);
3:3508,id3,0x200+3,set_motor_voltage( 0x200,0,0,motor_output[0],0,&hcan1);
4:6020,id4,0x205+3,set_motor_voltage(0x1FF,0, 0, 0, motor_output[0], &hcan2);
*/

//数据接收数组
uint8_t can_rx_flag=0;
uint8_t can_read_flag=0;

void can1_rx(void const * argument){

    /* USER CODE BEGIN can1_rx */
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 

    /* Infinite loop */
    moto_info_t motor_info_local[MOTOR_MAX_NUM];
    int16_t motor_output[5] = {0}; // 用于存储PID计算结果
    uint8_t control_flag=0;
    for(;;)
    {
        // 1. 使用vTaskDelayUntil实现精准的周期性延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if(can_rx_flag){
            can_read_flag=1;     //互斥锁
			motor_info_local[0].rotor_angle=motor_info[0].rotor_angle;
			//motor_info_local[0].rotor_speed=motor_info[0].rotor_speed;
			motor_info_local[4].rotor_angle=motor_info[4].rotor_angle;
			//motor_info_local[4].rotor_speed=motor_info[4].rotor_speed;
			can_read_flag=0;
            
            #if can_send_rx
            if(RC_CtrlData.remote.s2==1){
                vofa_send(2,(float)motor_info_local[motor_id].rotor_angle,(float)motor_info_local[motor_id].rotor_speed);
            }
            #endif
            can_rx_flag=0;
            //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
        }
        if(mpu_rx_flag){
            if(!mpu_data_busy){
                mpu_data_busy=true;
                motor_info_local[0].rotor_speed=mpu_data_global.gy;
                motor_info_local[4].rotor_speed=mpu_data_global.gx;
                mpu_data_busy=false;
            }        
        }
        // 调用PID任务，不再需要传递rx_flag
        motor_output[motor_id] = pid_speed_task(motor_info_local[motor_id].rotor_speed, motor_info_local[motor_id].rotor_angle);
        control_flag=0;	
        if(g_robot_control_state == CONTROL_ENABLED) {
            switch (motor_id)
            {
            case 0:
                set_motor_voltage(0x1FF,0, motor_output[0], 0, 0, &hcan1);
                break;
            case 4:
                set_motor_voltage(0x1FF,0, 0, 0, motor_output[4], &hcan2);
                break;    
            default:
                break;
            }          
            control_flag=1;	
        }   

        #if can_send_pid
        if(RC_CtrlData.remote.s2==3){
            #if pid_speed_mode 	
                vofa_send(4,(float)pid_speed.target,(float)pid_speed.now,(float)(pid_speed.now - pid_speed.target),(float)pid_speed.output);
            #else
                switch (motor_id)
                {
                case 0:
                    vofa_send(4,(float)pid_angle.target ,(float)pid_angle.now ,(float)(-pitch_global) ,(float)pid_angle.output);
                    break;
                case 4:
                vofa_send(4,(float)pid_angle.target ,(float)pid_angle.now ,(float)(yaw_global) ,(float)pid_angle.output);
                    break;    
                default:
                    break;
                }          
                
            #endif
        }
        #endif	
        
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
        if(can_read_flag)//简易互斥锁,忽略多余信息
        {
        memset(rx_data,0,sizeof(rx_data));
        return;
        }
        if ((rx_header.StdId == (FEEDBACK_ID_BASE+1))) //6020,id1,pitch
        {
			can_cnt ++;
			motor_info[0].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//motor_info[0].id= rx_header.StdId - FEEDBACK_ID_BASE; //motor_info[0].torque_current = ((rx_data[4] << 8) | rx_data[5]); 
			motor_info[0].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//motor_info[0].dlc=rx_header.DLC;//motor_info[0].temp           =   rx_data[6];
            can_rx_flag=1;
        }
        if(rx_header.StdId <= (CAN_3510Moto_ID+3) 
        && rx_header.StdId>=(CAN_3510Moto_ID+1)
        )
        {
        can_cnt ++;
        uint8_t index = rx_header.StdId-CAN_3510Moto_ID;//rx_header.StdId - FEEDBACK_ID_BASE; // get motor index by can_id
        motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
        motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
        }
    }
  
    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
        if(can_read_flag)//简易互斥锁,忽略多余信息
        {
        memset(rx_data,0,sizeof(rx_data));
        return;
        }
        if ((rx_header.StdId == (FEEDBACK_ID_BASE+3))) //6020,id4,yaw
        {
            can_cnt ++;
            motor_info[4].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//motor_info[0].id= rx_header.StdId - FEEDBACK_ID_BASE; //motor_info[0].torque_current = ((rx_data[4] << 8) | rx_data[5]); 
            motor_info[4].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//motor_info[0].dlc=rx_header.DLC;//motor_info[0].temp           =   rx_data[6];
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


