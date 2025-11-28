#include "ble.h"

#include "stdio.h"
#include "task_self.h"
#include "stm32f4xx_hal.h"


uint8_t ble_rx_buffer[ble_rx_buffer_size];
uint8_t sbus_rx_buf[SBUS_FRAME_SIZE];
uint8_t ble_tx_buffer[BLE_TX_BUF_LEN];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if ( huart->Instance== USART1)
   {
		//解析串口调试命令,控制pid参数和目标值,呼吸灯频率
		ble_parse(ble_rx_buffer);
		if(led_freq<=0.005 || led_freq>led_timer){
			led_freq=1; // 这个赋值是原子的，所以不需要互斥锁
		}
        // 重启DMA接收
		#if ble_send_rx_buf_debug
			ble_print(ble_rx_buffer,ble_rx_buffer_size);
		#endif
		HAL_UART_Receive_DMA(ble_uart, ble_rx_buffer, ble_rx_buffer_size);
   }

   if ( huart->Instance== USART2)
   {
		sbus_rx_flag=true; 
        // 重启DMA接收
		#if sbus_send_rx_buf_debug
			ble_print(sbus_rx_buf,SBUS_FRAME_SIZE);
			//my_printf("r:%s\n",sbus_rx_buf);
		#endif
		HAL_UART_Receive_DMA(sbus_uart, sbus_rx_buf, SBUS_FRAME_SIZE);
   }
}


// 任务/普通上下文调用：尝试使用 DMA，多次尝试/回退到阻塞发送以保证可靠性
void ble_print(uint8_t* buf,uint16_t len)
{
	if (len == 0 || buf == NULL) return;
	if (len > BLE_TX_BUF_LEN) len = BLE_TX_BUF_LEN;

	// 首次尝试使用 DMA
	if (HAL_UART_Transmit_DMA(ble_uart, buf, len) == HAL_OK) {
		return;
	} 
	/*
	// 如果 DMA 忙，尝试中止当前传输并重试（任务上下文允许调用阻塞操作）
	if (HAL_UART_AbortTransmit(ble_uart) == HAL_OK) {
		// 给硬件一点时间完成中止（很短），然后重试
		//HAL_Delay(1);
		for(uint8_t i=0;i<10;i++) {
			i++;
			i--;
		}
		if (HAL_UART_Transmit_DMA(ble_uart, buf, len) == HAL_OK) {
			return;
		}
	}
	*/
	// 最后兜底：阻塞发送，确保调试信息不会完全丢失	
	HAL_UART_Transmit(ble_uart, (uint8_t *) buf, len, 100);
}
void ble_Init(void)
{
	//my_printf("uart1_start\n");
	ble_print((uint8_t*)"start\n",5);
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
 	if ( HAL_UART_Receive_DMA(sbus_uart, sbus_rx_buf, SBUS_FRAME_SIZE)!= HAL_OK){
		// 错误处理
		#if ble_uart_send_debug
			my_printf("uart2_error\n");
		#endif
		Error_Handler();
	}
}


//约定发送字符串格式如下
//"St+1000.0000E"
//"0123456789012"
void ble_parse(uint8_t *buf)
{
    pid_pos *pid;
	pid=&pid_speed_yaw;
    switch (ble_control_id_global)
    {
    case  0:
        pid=&pid_speed_yaw;
        break;
    case  1:
        pid=&pid_angle_yaw;
        break;
    case  2:
        pid=&pid_speed_pitch;
        break;
    case  3:
        pid=&pid_angle_pitch;
        break;
    case  4:
        pid=&pid_speed_left_whell;
        break;
    case  5:
        pid=&pid_speed_right_whell;
        break;
    case  6:
        pid=&pid_speed_bopandianji;
        break;
    case  7:
        pid=&pid_angle_bopamdianji;
        break;
    default:
        return;
    }
	
	if(buf==NULL||buf[0] != 'S'  || buf[7] != '.' || buf[12] != 'E' ) 
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
            case 'g':
                ble_control_id= (uint8_t)val;
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
    // 拷贝所有float参数到缓冲区
    for (int i = 0; i < num; i++) {
        float value = va_arg(args, double); // float在可变参数中会自动提升为double
        memcpy(&ble_tx_buffer[i * 4], &value, 4);
    }
    
    va_end(args);
    
    // 添加帧尾 0x00, 0x00, 0x80, 0x7F
    uint8_t tail = num * 4;
    ble_tx_buffer[tail] = 0x00;
    ble_tx_buffer[tail + 1] = 0x00;
    ble_tx_buffer[tail + 2] = 0x80;
    ble_tx_buffer[tail + 3] = 0x7F;

    ble_print((uint8_t*)ble_tx_buffer, tail + 4);
}

void my_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    int len = vsnprintf((char*)ble_tx_buffer, BLE_TX_BUF_LEN, format, args);
    if (len > 0 && len < BLE_TX_BUF_LEN)
    {
      ble_print((uint8_t *)ble_tx_buffer,len);
    }

    va_end(args);
}






