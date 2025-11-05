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
osThreadId serial_tx_taskHandle;
osThreadId cmdparse_taskHandle;
osThreadId mpu6050_read_taHandle;
osMessageQId led_control_queueHandle;
osMessageQId ble_rx_queueHandle;
osMessageQId mpu_data_queueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void led_breath(void const * argument);
void ble_send(void const * argument);
void ble_receive(void const * argument);
void mpu6050_read(void const * argument);

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
  osMessageQDef(led_control_queue, 4, uint16_t);
  led_control_queueHandle = osMessageCreate(osMessageQ(led_control_queue), NULL);

  /* definition and creation of ble_rx_queue */
  osMessageQDef(ble_rx_queue, 4, 14);
  ble_rx_queueHandle = osMessageCreate(osMessageQ(ble_rx_queue), NULL);

  /* definition and creation of mpu_data_queue */
  osMessageQDef(mpu_data_queue, 4, 6);
  mpu_data_queueHandle = osMessageCreate(osMessageQ(mpu_data_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of breathing_led_t */
  osThreadDef(breathing_led_t, led_breath, osPriorityBelowNormal, 0, 128);
  breathing_led_tHandle = osThreadCreate(osThread(breathing_led_t), NULL);

  /* definition and creation of serial_tx_task */
  osThreadDef(serial_tx_task, ble_send, osPriorityLow, 0, 128);
  serial_tx_taskHandle = osThreadCreate(osThread(serial_tx_task), NULL);

  /* definition and creation of cmdparse_task */
  osThreadDef(cmdparse_task, ble_receive, osPriorityNormal, 0, 128);
  cmdparse_taskHandle = osThreadCreate(osThread(cmdparse_task), NULL);

  /* definition and creation of mpu6050_read_ta */
  osThreadDef(mpu6050_read_ta, mpu6050_read, osPriorityNormal, 0, 128);
  mpu6050_read_taHandle = osThreadCreate(osThread(mpu6050_read_ta), NULL);

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
    osDelay(1);
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

/* USER CODE BEGIN Header_ble_send */
/**
* @brief Function implementing the serial_tx_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ble_send */
__weak void ble_send(void const * argument)
{
  /* USER CODE BEGIN ble_send */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ble_send */
}

/* USER CODE BEGIN Header_ble_receive */
/**
* @brief Function implementing the cmdparse_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ble_receive */
__weak void ble_receive(void const * argument)
{
  /* USER CODE BEGIN ble_receive */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ble_receive */
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
