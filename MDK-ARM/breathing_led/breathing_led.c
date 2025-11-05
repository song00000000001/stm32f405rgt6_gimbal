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
	static uint32_t breath_period_ms = 1; 
	// 默认步长周期 1ms，乘以200/step=50后呼吸周期为50ms。20hz
	uint32_t new_period_from_queue;
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

        // 2. 更新亮度
        brightness += step;
        if (brightness >= 100) step = -4;
        if (brightness <= 0)   step = 4;
        
        // 3. 更新全局亮度变量 (供PWM中断使用)
        // 这个赋值是原子的，所以不需要互斥锁
		g_led_brightness = brightness; 
	  
        // 4. 根据当前周期延时
		osDelay(breath_period_ms);
 
  }
  /* USER CODE END led_breath */
}
