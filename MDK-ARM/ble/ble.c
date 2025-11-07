#include "ble.h"
#include "string.h"
#include "breathing_led.h"
#include "pid.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"

float led_freq=1;
bool sbus_receive_success=false;
bool sbus_rx_flag=false;
uint8_t  ble_rx_buffer[ble_rx_buffer_size];
uint8_t sbus_rx_buf[SBUS_FRAME_SIZE];
SbusData_t my_sbus_data;

extern osMessageQId ble_rx_queueHandle,led_control_queueHandle,sbus_buf_queueHandle;

void ble_parse(uint8_t *buf);
bool sbus_parse(const uint8_t* frame, SbusData_t* sbus_data);

void ble_Init(void)
{
	#if ble_uart_send_debug
		my_printf("start_uart\n");
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
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
		sbus_rx_flag=sbus_parse(sbus_rx_buf, &my_sbus_data);
		    
        // 重启DMA接收
		#if sbus_send_rx_buf_debug
			ble_print(sbus_rx_buf,SBUS_FRAME_SIZE);
		#endif
		HAL_UART_Receive_DMA(huart_sbus, sbus_rx_buf, SBUS_FRAME_SIZE);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
   }
}


void sbus_receive(void const * argument)
{
  /* USER CODE BEGIN sbus_receive */
	uint8_t local_rx_buffer[ble_rx_buffer_size];
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
  /* Infinite loop */
  for(;;)
  {
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
	// 1. 使用vTaskDelayUntil实现精准的周期性延时
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
	sbus_receive_success=false;
	if(sbus_rx_flag){//如果成功解析
		sbus_receive_success=!my_sbus_data.failsafe;//并且没有丢失联系就激活,原子操作
		#if sbus_send_chan
			my_printf("ch1:%d,failsafe:%d\r\n",my_sbus_data.channels[0],my_sbus_data.failsafe);
		#endif
	}
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
  }
  /* USER CODE END sbus_receive */
}

bool sbus_parse(const uint8_t* frame, SbusData_t* sbus_data)
{
    // 1. 帧头帧尾校验
    if (frame[0] != 0x0F || frame[24] != 0x00) {
        return false;
    }

    // 2. 解码16个通道的数据 (核心部分)
    // 使用位操作精确地提取每个11位通道
    sbus_data->channels[0]  = ((frame[1]    | frame[2] << 8) & 0x07FF);
    sbus_data->channels[1]  = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
    sbus_data->channels[2]  = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
    sbus_data->channels[3]  = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
    sbus_data->channels[4]  = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
    sbus_data->channels[5]  = ((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF);
    sbus_data->channels[6]  = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
    sbus_data->channels[7]  = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
    sbus_data->channels[8]  = ((frame[12]   | frame[13] << 8) & 0x07FF);
    sbus_data->channels[9]  = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
    sbus_data->channels[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
    sbus_data->channels[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
    sbus_data->channels[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
    sbus_data->channels[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
    sbus_data->channels[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
    sbus_data->channels[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);
    
    // 3. 解析标志位
    sbus_data->ch17 = frame[23] & 0x01;
    sbus_data->ch18 = frame[23] & 0x02;
    sbus_data->frame_lost = frame[23] & 0x04;
    sbus_data->failsafe = frame[23] & 0x08;

    return true;
}

//约定发送字符串格式如下
//"St+1000.0000E"
//"0123456789012"
void ble_parse(uint8_t *buf)
{
	#if 0
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

void ble_print(uint8_t* buf,uint16_t len)
{
	HAL_UART_Transmit_DMA(ble_uart,(uint8_t *) buf, len);
	//HAL_UART_Transmit(ble_uart, (uint8_t *) buf, len, 100);
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
