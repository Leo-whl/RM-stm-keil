/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
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
/* Definitions for led_task */
osThreadId_t led_taskHandle;
const osThreadAttr_t led_task_attributes = {
  .name = "led_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can_rx_task */
osThreadId_t can_rx_taskHandle;
const osThreadAttr_t can_rx_task_attributes = {
  .name = "can_rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dbus_task */
osThreadId_t dbus_taskHandle;
const osThreadAttr_t dbus_task_attributes = {
  .name = "dbus_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chasiss_task */
osThreadId_t chasiss_taskHandle;
const osThreadAttr_t chasiss_task_attributes = {
  .name = "chasiss_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gimbal_task */
osThreadId_t gimbal_taskHandle;
const osThreadAttr_t gimbal_task_attributes = {
  .name = "gimbal_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for bmi_task */
osThreadId_t bmi_taskHandle;
const osThreadAttr_t bmi_task_attributes = {
  .name = "bmi_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void led_task_entry(void *argument);
void can_rx_task_entry(void *argument);
void dbus_task_entry(void *argument);
void chasiss_task_entry(void *argument);
void gimbal_task_entry(void *argument);
void bmi_task_entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of led_task */
  led_taskHandle = osThreadNew(led_task_entry, NULL, &led_task_attributes);

  /* creation of can_rx_task */
  can_rx_taskHandle = osThreadNew(can_rx_task_entry, NULL, &can_rx_task_attributes);

  /* creation of dbus_task */
  dbus_taskHandle = osThreadNew(dbus_task_entry, NULL, &dbus_task_attributes);

  /* creation of chasiss_task */
  chasiss_taskHandle = osThreadNew(chasiss_task_entry, NULL, &chasiss_task_attributes);

  /* creation of gimbal_task */
  gimbal_taskHandle = osThreadNew(gimbal_task_entry, NULL, &gimbal_task_attributes);

  /* creation of bmi_task */
  bmi_taskHandle = osThreadNew(bmi_task_entry, NULL, &bmi_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_led_task_entry */
/**
  * @brief  Function implementing the led_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_led_task_entry */
__weak void led_task_entry(void *argument)
{
  /* USER CODE BEGIN led_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_task_entry */
}

/* USER CODE BEGIN Header_can_rx_task_entry */
/**
* @brief Function implementing the can_rx_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_rx_task_entry */
__weak void can_rx_task_entry(void *argument)
{
  /* USER CODE BEGIN can_rx_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END can_rx_task_entry */
}

/* USER CODE BEGIN Header_dbus_task_entry */
/**
* @brief Function implementing the dbus_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dbus_task_entry */
__weak void dbus_task_entry(void *argument)
{
  /* USER CODE BEGIN dbus_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dbus_task_entry */
}

/* USER CODE BEGIN Header_chasiss_task_entry */
/**
* @brief Function implementing the chasiss_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chasiss_task_entry */
__weak void chasiss_task_entry(void *argument)
{
  /* USER CODE BEGIN chasiss_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chasiss_task_entry */
}

/* USER CODE BEGIN Header_gimbal_task_entry */
/**
* @brief Function implementing the gimbal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task_entry */
__weak void gimbal_task_entry(void *argument)
{
  /* USER CODE BEGIN gimbal_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task_entry */
}

/* USER CODE BEGIN Header_bmi_task_entry */
/**
* @brief Function implementing the bmi_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bmi_task_entry */
__weak void bmi_task_entry(void *argument)
{
  /* USER CODE BEGIN bmi_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END bmi_task_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

