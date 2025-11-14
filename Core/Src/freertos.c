/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId breathing_led_tHandle;
osThreadId mpu_read_taskHandle;
osThreadId can1_rx_taskHandle;
osThreadId sbus_rx_taskHandle;
osThreadId bopan_motor_tasHandle;
osMessageQId led_control_queueHandle;
uint8_t led_control_queueBuffer[ 5 * 2 ];
osStaticMessageQDef_t led_control_queueControlBlock;
osMessageQId can_rx_queueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void led_breath(void const * argument);
void mpu6050_read(void const * argument);
void can1_rx(void const * argument);
void sbus_receive(void const * argument);
void bopan_motor_set(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of led_control_queue */
  osMessageQStaticDef(led_control_queue, 5, 2, led_control_queueBuffer, &led_control_queueControlBlock);
  led_control_queueHandle = osMessageCreate(osMessageQ(led_control_queue), NULL);

  /* definition and creation of can_rx_queue */
  osMessageQDef(can_rx_queue, 5, 9);
  can_rx_queueHandle = osMessageCreate(osMessageQ(can_rx_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of breathing_led_t */
  osThreadDef(breathing_led_t, led_breath, osPriorityIdle, 0, 128);
  breathing_led_tHandle = osThreadCreate(osThread(breathing_led_t), NULL);

  /* definition and creation of mpu_read_task */
  osThreadDef(mpu_read_task, mpu6050_read, osPriorityNormal, 0, 128);
  mpu_read_taskHandle = osThreadCreate(osThread(mpu_read_task), NULL);

  /* definition and creation of can1_rx_task */
  osThreadDef(can1_rx_task, can1_rx, osPriorityAboveNormal, 0, 128);
  can1_rx_taskHandle = osThreadCreate(osThread(can1_rx_task), NULL);

  /* definition and creation of sbus_rx_task */
  osThreadDef(sbus_rx_task, sbus_receive, osPriorityHigh, 0, 128);
  sbus_rx_taskHandle = osThreadCreate(osThread(sbus_rx_task), NULL);

  /* definition and creation of bopan_motor_tas */
  osThreadDef(bopan_motor_tas, bopan_motor_set, osPriorityBelowNormal, 0, 128);
  bopan_motor_tasHandle = osThreadCreate(osThread(bopan_motor_tas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_led_breath */
/**
* @brief Function implementing the breathing_led_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_breath */
__weak void led_breath(void const * argument)
{
  /* USER CODE BEGIN led_breath */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_breath */
}

/* USER CODE BEGIN Header_mpu6050_read */
/**
* @brief Function implementing the mpu6050_read_ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mpu6050_read */
__weak void mpu6050_read(void const * argument)
{
  /* USER CODE BEGIN mpu6050_read */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END mpu6050_read */
}

/* USER CODE BEGIN Header_can1_rx */
/**
* @brief Function implementing the can1_rx_task thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_can1_rx */
__weak void can1_rx(void const * argument)
{
  /* USER CODE BEGIN can1_rx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can1_rx */
}

/* USER CODE BEGIN Header_sbus_receive */
/**
* @brief Function implementing the sbus_rx_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sbus_receive */
__weak void sbus_receive(void const * argument)
{
  /* USER CODE BEGIN sbus_receive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END sbus_receive */
}

/* USER CODE BEGIN Header_bopan_motor_set */
/**
* @brief Function implementing the bopan_motor_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bopan_motor_set */
__weak void bopan_motor_set(void const * argument)
{
  /* USER CODE BEGIN bopan_motor_set */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END bopan_motor_set */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
