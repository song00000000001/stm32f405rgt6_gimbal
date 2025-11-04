#include "ble.h"
#include "string.h"
#include "breathing_led.h"
#include "pid.h"

uint8_t  ble_rx_buffer[ble_rx_buffer_size];//串口接收缓冲区
uint8_t  ble_txBuffer[64];//串口发送缓冲区

void ble_Init(void)
{
	
	if (HAL_UART_Receive_DMA(ble_uart, ble_rx_buffer, ble_rx_buffer_size) != HAL_OK)//手动打开第一次串口接收,并**同时**检查状态。第二次会返回busy。
  {
    // 错误处理
    Error_Handler();
  }
	
}

void ble_print(uint8_t* buf,uint16_t len)
{
	HAL_UART_Transmit_DMA(ble_uart,(uint8_t *) buf, len);
}

//约定发送字符串格式如下
//"St+1000.0000E"
//"0123456789012"

void BLE_ParsePID_pos(struct pid_pos* pid)
{	
  uint8_t buf[ble_rx_buffer_size];
	memcpy(buf,ble_rx_buffer,ble_rx_buffer_size);
  if(buf[0] != 'S' || buf[7] != '.' || buf[12] != 'E' ) 
			return;
	
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
		
		if(val<-180) 
			val=-180;
		if(val>=180) 
			val=180;

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
        default:
            break;
    }
}

#include <stdarg.h>
#include <string.h>

void vofa_send(int num, ...) {
    va_list args;
    va_start(args, num);
    
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

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"

extern osMessageQId ble_rx_queueHandle;

void HAL_ble_rxCpltCallback(UART_HandleTypeDef *huart)
{
   if ( huart->Instance== USART1)
   {
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		 
				// 将接收到的数据buffer指针发送到队列
        xQueueSendFromISR(ble_rx_queueHandle, &ble_rx_buffer, &xHigherPriorityTaskWoken);
        
        // 重启DMA接收
        HAL_UART_Receive_DMA(&huart1, ble_rx_buffer, sizeof(ble_rx_buffer));
        
        // 如果有更高优先级的任务被唤醒，进行上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		 
       
        //处理完数据后，必须重新开启下一次接收
        HAL_UART_Receive_IT(&huart1, ble_rx_buffer, ble_rx_buffer_size);
   }
}

void ble_receive_handle(uint8_t *buf,uint8_t * f)
{
	
	 if(buf[0] != 'S'  || buf[7] != '.' || buf[12] != 'E' ) 
   {
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
					case 'L':
							*f=(uint8_t)val;
							break;
					default:
							break;
				}
		}
		//HAL_UART_Transmit(&huart1, ble_rx_buffer, ble_rx_buffer_size, 100); //串口发送字符串
		//memset(ble_rx_buffer, 0, ble_rx_buffer_size);
}