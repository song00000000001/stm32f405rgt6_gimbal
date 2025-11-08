#include "ble.h"
#include "string.h"
#include "breathing_led.h"
#include "pid.h"
#include "stdio.h"
#include <stdlib.h>
#include "stm32f4xx_it.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"

float led_freq=1;
bool sbus_receive_success=false;
bool sbus_rx_flag=false;
bool sbus_read_flag=false;
uint8_t ble_rx_buffer[ble_rx_buffer_size];
uint8_t sbus_rx_buf[SBUS_FRAME_SIZE];
uint8_t sbus_rx_buf_t[SBUS_FRAME_SIZE];

extern osMessageQId led_control_queueHandle;

void ble_parse(uint8_t *buf);
bool sbus_parse(const uint8_t* frame, SbusData_t* sbus_data);
	  
void ble_print(uint8_t* buf,uint16_t len)
{
	//HAL_UART_Transmit_DMA(ble_uart,(uint8_t *) buf, len);
	HAL_UART_Transmit(ble_uart, (uint8_t *) buf, len, 10);
}


void ble_Init(void)
{
	my_printf("uart1_start\n");
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
    const TickType_t xFrequency = pdMS_TO_TICKS(10); //每100hz检查遥控信号
	SbusData_t my_sbus_data;
  /* Infinite loop */
  for(;;)
  {
	// 1. 使用vTaskDelayUntil实现精准的周期性延时
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
	 
	 //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
	sbus_receive_success=false;
	
	if(sbus_rx_flag){//如果中断收到信息
		sbus_read_flag=true;//打开锁,防止数据覆盖。此时有一帧完整的旧数据可读取。
		sbus_rx_flag=sbus_parse(sbus_rx_buf_t, &my_sbus_data);//解析成功后才执行命令
		sbus_read_flag=false;	//解开锁，中断会向2号缓冲区搬数据
  	}	

	if(sbus_rx_flag){//如果成功解析
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
		sbus_receive_success=((!my_sbus_data.failsafe)  && (my_sbus_data.ch1>1500));//并且没有丢失联系,并且遥感在上,就激活(原子操作
		#if sbus_send_chan
			if(HAL_DMA_GetState(&hdma_usart1_tx)==HAL_DMA_STATE_READY){
				my_printf("ch1:%4dch2:%4dch3:%4dch4:%4dsw1:%dsw2:%d,l:%d,f:%d\n\0",
					my_sbus_data.ch1, my_sbus_data.ch2,
					my_sbus_data.ch3,my_sbus_data.ch4,
					my_sbus_data.sw1,my_sbus_data.sw2,
					my_sbus_data.frame_lost,my_sbus_data.failsafe);  
			} 
		#endif
		sbus_rx_flag=false;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
	}

  }
  /* USER CODE END sbus_receive */
}

/*
if(HAL_DMA_GetState(&hdma_usart1_tx)==HAL_DMA_STATE_READY){
my_printf("c1:%4dc2:%4dc3:%4dc4:%4d"
	"c5:%4dc6:%4dc7:%4dc8:%4drs:%4d"
	"lost:%d,fail:%d\r\n",
	my_sbus_data.channels[0], my_sbus_data.channels[1],
	my_sbus_data.channels[2], my_sbus_data.channels[3],
	my_sbus_data.channels[4], my_sbus_data.channels[5],
	my_sbus_data.channels[6], my_sbus_data.channels[7],
	my_sbus_data.channels[15],
	my_sbus_data.frame_lost,my_sbus_data.failsafe); 
} 
*/

bool sbus_parse(const uint8_t* frame, SbusData_t* sbus_data)
{
	// 1. 帧头帧尾校验
	if (frame[0] != 0x0F || frame[24] != 0x00) {
		return false;
	}

    // 2. 解码dbus
	//satori：这里完成的是数据的分离和拼接，减去1024是为了让数据的中间值变为0
	sbus_data->ch1 = (frame[0] | frame[1] << 8) & 0x07FF;
	sbus_data->ch1 -= 1024;
	sbus_data->ch2 = (frame[1] >> 3 | frame[2] << 5) & 0x07FF;
	sbus_data->ch2 -= 1024;
	sbus_data->ch3 = (frame[2] >> 6 | frame[3] << 2 | frame[4] << 10) & 0x07FF;
	sbus_data->ch3 -= 1024;
	sbus_data->ch4 = (frame[4] >> 1 | frame[5] << 7) & 0x07FF;
	sbus_data->ch4 -= 1024;
	
	//satori:防止数据零漂，设置正负5的死区
	/* prevent remote control zero deviation */
	if(sbus_data->ch1 <= 5 && sbus_data->ch1 >= -5)
		sbus_data->ch1 = 0;
	if(sbus_data->ch2 <= 5 && sbus_data->ch2 >= -5)
		sbus_data->ch2 = 0;
	if(sbus_data->ch3 <= 5 && sbus_data->ch3 >= -5)
		sbus_data->ch3 = 0;
	if(sbus_data->ch4 <= 5 && sbus_data->ch4 >= -5)
		sbus_data->ch4 = 0;
	
	sbus_data->sw1 = ((frame[5] >> 4) & 0x000C) >> 2;
	sbus_data->sw2 = (frame[5] >> 4) & 0x0003;
	
	//satori:防止数据溢出
	if ((abs(sbus_data->ch1) > 660) || \
		(abs(sbus_data->ch2) > 660) || \
		(abs(sbus_data->ch3) > 660) || \
		(abs(sbus_data->ch4) > 660))
	{
		memset(sbus_data, 0, sizeof( SbusData_t));
		return false;
	}

	sbus_data->mouse.x = frame[6] | (frame[7] << 8); // x axis
	sbus_data->mouse.y = frame[8] | (frame[9] << 8);
	sbus_data->mouse.z = frame[10] | (frame[11] << 8);

	sbus_data->mouse.l = frame[12];
	sbus_data->mouse.r = frame[13];

	sbus_data->kb.key_code = frame[14] | frame[15] << 8; // key borad code
	sbus_data->wheel = (frame[16] | frame[17] << 8) - 1024;

    sbus_data->frame_lost = frame[23] & 0x04;
    sbus_data->failsafe = frame[23] & 0x08;

    return true;
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
