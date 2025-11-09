#include "ble.h"
#include "string.h"
#include "breathing_led.h"
#include "pid.h"
#include "stdio.h"
#include <stdlib.h>
#include "bsp_can.h"
#include "stm32f4xx_it.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"

float led_freq=1;
uint8_t sbus_receive_success=false;
bool sbus_rx_flag=false;
bool sbus_read_flag=false;
bool sbus_read_fine_flag=false;
uint8_t ble_rx_buffer[ble_rx_buffer_size];
uint8_t sbus_rx_buf[SBUS_FRAME_SIZE];
uint8_t sbus_rx_buf_t[SBUS_FRAME_SIZE];
volatile ControlState_t g_robot_control_state = CONTROL_DISABLED;

extern osMessageQId led_control_queueHandle;

void ble_parse(uint8_t *buf);
bool sbus_parse(const uint8_t* frame, SbusData_t* sbus_data);
	  
void ble_print(uint8_t* buf,uint16_t len)
{
	#if 0
		HAL_UART_Transmit_DMA(ble_uart,(uint8_t *) buf, len);
	#else
		HAL_UART_Transmit(ble_uart, (uint8_t *) buf, len, 10);
	#endif
}


void ble_Init(void)
{
	//my_printf("uart1_start\n");
	ble_print((uint8_t*)"start",5);
	#if ble_uart_send_debug
		my_printf("start_uart1\n");
	#endif
	//手动打开第一次串口接收,并**同时**检查状态。第二次会返回busy。
	if (HAL_UART_Receive_DMA(ble_uart, ble_rx_buffer, ble_rx_buffer_size) != HAL_OK){
    	// 错误处理
		#if ble_uart_send_debug
			my_printf("uart1_error\n");
		#endif
		Error_Handler();
  	}
 	if ( HAL_UART_Receive_DMA(huart_sbus, sbus_rx_buf, SBUS_FRAME_SIZE)!= HAL_OK){
		// 错误处理
		#if ble_uart_send_debug
			my_printf("uart2_error\n");
		#endif
		Error_Handler();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if ( huart->Instance== USART1)
   {
		ble_parse(ble_rx_buffer);
		if(led_freq<=0.005 || led_freq>led_timer){
			led_freq=1; // 这个赋值是原子的，所以不需要互斥锁
		}
        // 重启DMA接收
		#if ble_send_rx_buf_debug
			ble_print(ble_rx_buffer,ble_rx_buffer_size);
		#endif
		HAL_UART_Receive_DMA(&huart1, ble_rx_buffer, ble_rx_buffer_size);
   }

   if ( huart->Instance== USART2)
   {

		if(!sbus_read_flag) memcpy( sbus_rx_buf_t,  sbus_rx_buf,SBUS_FRAME_SIZE);
		sbus_rx_flag=true; 
        // 重启DMA接收
		#if sbus_send_rx_buf_debug
			ble_print(sbus_rx_buf,SBUS_FRAME_SIZE);
			//my_printf("r:%s\n",sbus_rx_buf);
		#endif

		HAL_UART_Receive_DMA(huart_sbus, sbus_rx_buf, SBUS_FRAME_SIZE);
   }
}

  /*

*/

void sbus_receive(void const * argument)
{
  /* USER CODE BEGIN sbus_receive */
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
    const TickType_t xFrequency = pdMS_TO_TICKS(14); //每70hz检查遥控信号
	SbusData_t my_sbus_data;
	uint8_t fail_count=0,fail_counter2=0;
	static uint16_t suc_counter=0;
  /* Infinite loop */
  for(;;)
  {
	// 1. 使用vTaskDelayUntil实现精准的周期性延时
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
	 
	 //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
	if(sbus_rx_flag){//如果中断收到信息
		sbus_read_flag=true;//打开锁,防止数据覆盖。此时有一帧完整的旧数据可读取。
		Get_DR16_Data(sbus_rx_buf_t);
		sbus_read_flag=false;	//解开锁，中断会向2号缓冲区搬数据
		
		if(RC_CtrlData.remote.s1 == 3) {
			g_robot_control_state = CONTROL_ENABLED;
			LED_GREEN_ON(); // 激活时常亮，更直观
			fail_counter2=0;
		} 
		else {
			 fail_counter2++;
			if(fail_counter2>=2){
				g_robot_control_state = CONTROL_DISABLED;
				LED_GREEN_OFF(); 
				fail_counter2=0;
			}
		}

		#if sbus_send_chan
		if(HAL_DMA_GetState(&hdma_usart1_tx)==HAL_DMA_STATE_READY&&RC_CtrlData.remote.s2==2){
			vofa_send(3,(float)RC_CtrlData.remote.ch0,(float)RC_CtrlData.remote.s1,(float)RC_CtrlData.remote.s2);
		}
		#endif
		sbus_rx_flag=false;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
		fail_count=0;
  	}
	else{
		fail_count++;
		if(fail_count>=2){
			g_robot_control_state = CONTROL_DISABLED;
			LED_GREEN_OFF();
			fail_count = 0;
		}
	}		

  }
  /* USER CODE END sbus_receive */
}


//约定发送字符串格式如下
//"St+1000.0000E"
//"0123456789012"
void ble_parse(uint8_t *buf)
{
	#if pid_speed_mode
	    pid_inc *pid=&pid_speed;
	#else
	    pid_pos *pid=&pid_angle;
	#endif
	if(buf[0] != 'S'  || buf[7] != '.' || buf[12] != 'E' ) 
   	{
		//memset(ble_rx_buffer, 0, ble_rx_buffer_size);
		return;
   	}
	else{
		float val = 
			(buf[3] - '0') *1000 +
			(buf[4] - '0') *100+
			(buf[5] - '0') *10 +
			(buf[6] - '0') +
			(buf[8] - '0') *0.1f +
			(buf[9] - '0') *0.01f +
			(buf[10] - '0') *0.001f +
			(buf[11] - '0') *0.0001f;

		if(buf[2] == '+')
				val = val;
		else if(buf[2] == '-')
				val = -val;
		else
				return;

		if(val<-1000) 
				val=-1000;
		if(val>=1000) 
				val=1000;
		
		switch(buf[1])
		{
			case 'p':
            	pid->Kp = val;
				break;
			case 'i':
				pid->Ki = val;
				break;
			case 'd':
				pid->Kd = val;
				break;
			case 't':
				pid->target = val;
				break;
			case 'L':
				led_freq=val;
				break;
			default:
					break;
		}
	}
	//memset(ble_rx_buffer, 0, ble_rx_buffer_size);
}

#include <stdarg.h>
void vofa_send(int num, ...) {
    va_list args;
    va_start(args, num);
    uint8_t  ble_txBuffer[BLE_TX_BUF_LEN];
    // 拷贝所有float参数到缓冲区
    for (int i = 0; i < num; i++) {
        float value = va_arg(args, double); // float在可变参数中会自动提升为double
        memcpy(&ble_txBuffer[i * 4], &value, 4);
    }
    
    va_end(args);
    
    // 添加帧尾 0x00, 0x00, 0x80, 0x7F
    uint8_t tail = num * 4;
    ble_txBuffer[tail] = 0x00;
    ble_txBuffer[tail + 1] = 0x00;
    ble_txBuffer[tail + 2] = 0x80;
    ble_txBuffer[tail + 3] = 0x7F;

    ble_print((uint8_t*)ble_txBuffer, tail + 4);
}

void my_printf(const char *format, ...)
{
    char buf[BLE_TX_BUF_LEN];
    va_list args;
    va_start(args, format);

    int len = vsnprintf(buf, BLE_TX_BUF_LEN, format, args);
    if (len > 0 && len < BLE_TX_BUF_LEN)
    {
      ble_print((uint8_t *)buf,len);
    }

    va_end(args);
}






