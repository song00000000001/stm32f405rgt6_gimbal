#include "ble.h"
#include "string.h"
#include "breathing_led.h"
#include "pid.h"

uint8_t  uart1_buf[uart1_buf_size];//串口接收缓冲区
uint8_t  ble_txBuffer[64];//串口发送缓冲区
uint8_t  test_buf[]="hello world\n";//串口发送测试字符串

void ble_Init(void)
{
	
	//HAL_UART_Transmit(ble_uart, test_buf, sizeof(test_buf)/sizeof(uint8_t)-1, 100); //串口发送测试字符串
	HAL_UART_Transmit_DMA(ble_uart, test_buf, sizeof(test_buf)/sizeof(uint8_t)-1); //串口发送测试字符串
	if (HAL_UART_Receive_DMA(ble_uart, uart1_buf, uart1_buf_size) != HAL_OK)//手动打开第一次串口接收,并**同时**检查状态。第二次会返回busy。
  {
    // 错误处理
		HAL_UART_Receive_DMA(ble_uart,(uint8_t *) "uart1_error\n",12); 
		//HAL_UART_Transmit(ble_uart,(uint8_t *) "uart1_error\n",12, 100); 
    Error_Handler();
  }
}

void ble_print(uint8_t* buf,uint16_t len)
{
	//HAL_UART_Transmit(ble_uart,(uint8_t *) buf, len, 100);
	HAL_UART_Transmit_DMA(ble_uart,(uint8_t *) buf, len);
}

//约定发送字符串格式如下
//"St+1000.0000E"
//"0123456789012"

void BLE_ParsePID_pos(struct pid_pos* pid)
{	
  uint8_t buf[uart1_buf_size];
	memcpy(buf,uart1_buf,uart1_buf_size);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if ( huart->Instance== USART1)
   {
        if(uart1_buf[0] != 'S'  || uart1_buf[7] != '.' || uart1_buf[12] != 'E' ) 
        {
					memset(uart1_buf, 0, uart1_buf_size);
        }
				else{
					//业务逻辑
					BLE_ParsePID_pos(&pid_angle);
					uart1_buf[13]='\0';
				}
				//HAL_UART_Transmit(&huart1, uart1_buf, uart1_buf_size, 100); //串口发送字符串
				//memset(uart1_buf, 0, uart1_buf_size);
        //处理完数据后，必须重新开启下一次接收
        HAL_UART_Receive_IT(&huart1, uart1_buf, uart1_buf_size);
   }
}


// 错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 判断是否是 Overrun Error
        if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
        {
            // ORE 错误发生后，HAL库会自动停止接收。
            // 我们需要在这里手动清除ORE标志位，并重新启动接收。
            
            // F1系列HAL库的ORE标志位需要先读SR再读DR来清除
            __HAL_UART_CLEAR_OREFLAG(huart);
            
            // 重新开启接收
            // 注意：这里不能用 Abort 函数，因为它会触发另一次ErrorCallback导致死循环
            // 最稳妥的方式是直接重新调用接收函数
            HAL_UART_Receive_IT(&huart1, uart1_buf, uart1_buf_size);
        }
    }
}
