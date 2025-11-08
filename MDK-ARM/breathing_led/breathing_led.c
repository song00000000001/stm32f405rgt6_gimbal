#include "breathing_led.h"

volatile uint16_t g_led_brightness = 0;

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "ble.h"

extern osMessageQId led_control_queueHandle;

void led_breath(void const * argument)
{
  /* USER CODE BEGIN led_breath */
	static uint32_t breath_period_ms = led_timer; 
	uint16_t new_period_from_queue = 0;
	static uint8_t brightness = 0;
	static int8_t step = 4;
  /* Infinite loop */
  for(;;)
  {
	  
	  // 1. 非阻塞地检查是否有新的周期设置
      if (xQueueReceive(led_control_queueHandle, &new_period_from_queue, 0) == pdPASS)
      {
        if (new_period_from_queue > 0 && new_period_from_queue < 200)
        {
            breath_period_ms = new_period_from_queue; // 更新内部状态
        }
      }
      //f=1hz,则需要延时20*50ms=1s,f_max=50hz,延时20*1=20ms

		  breath_period_ms =led_timer/led_freq;
      // 2. 更新亮度
      brightness += step;
      if (brightness >= 100) step = -5;
      if (brightness <= 0)   step = 5;
        
      // 3. 更新全局亮度变量 (供PWM中断使用)
      // 这个赋值是原子的，所以不需要互斥锁
	    g_led_brightness = brightness; 
	  
      // 4. 根据当前周期延时
	    osDelay(breath_period_ms);//100/5=20,20*50=1000
 
  }
  /* USER CODE END led_breath */
}
