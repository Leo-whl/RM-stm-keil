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
#include "tim.h"
#include "stm32f4xx.h"
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
osThreadId LED_TaskHandle;
osThreadId Chassis_TaskHandle;
osThreadId Task_threeHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LED_Task_entry(void const * argument);
void Chassis_Task_entry(void const * argument);
void Three_entry(void const * argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, LED_Task_entry, osPriorityNormal, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, Chassis_Task_entry, osPriorityIdle, 0, 128);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of Task_three */
  osThreadDef(Task_three, Three_entry, osPriorityIdle, 0, 128);
  Task_threeHandle = osThreadCreate(osThread(Task_three), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LED_Task_entry */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
void aRGB_led_show(uint32_t aRGB)
{
  static uint8_t alpha;
  static uint16_t red,green,blue;

  alpha = (aRGB & 0xFF000000) >> 24;
  red = ((aRGB & 0x00FF0000) >> 16) * alpha;
  green = ((aRGB & 0x0000FF00) >> 8) * alpha;
  blue = ((aRGB & 0x000000FF) >> 0) * alpha;

  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
(((__CHANNEL__) == TIM_CHANNEL_1) ? ((__HANDLE__)->Instance->CCR1 = (__COMPARE__)) :\
 ((__CHANNEL__) == TIM_CHANNEL_2) ? ((__HANDLE__)->Instance->CCR2 = (__COMPARE__)) :\
 ((__CHANNEL__) == TIM_CHANNEL_3) ? ((__HANDLE__)->Instance->CCR3 = (__COMPARE__)) :\
 ((__HANDLE__)->Instance->CCR4 = (__COMPARE__)))
/* USER CODE END Header_LED_Task_entry */
__weak void LED_Task_entry(void const * argument)
{
  /* USER CODE BEGIN LED_Task_entry */

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  /* Infinite loop */
  for(;;)
  {
    aRGB_led_show(0x7F123456);
    osDelay(1000);
    aRGB_led_show(0xffff0000);
    osDelay(1000);
    aRGB_led_show(0xFFFFFF56);
    osDelay(1000);

  }
  /* USER CODE END LED_Task_entry */
}

/* USER CODE BEGIN Header_Chassis_Task_entry */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
#include "can_r.h"
#include "pid.h"

int16_t led_cnt;
int16_t text_speed = 0;
int16_t target_yaw_speed;
float target_yaw_angle = 0;
float now_yaw_angle;
extern moto_info_t motor_yaw_info;
extern pid_struct_t gimbal_yaw_speed_pid;
extern pid_struct_t gimbal_yaw_angle_pid;


double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi�?
{
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
int trigger_speed = 0;
int shoot_speed_1= 0;    //-16384—�??16834
int shoot_speed_2= 0;    //-16384—�??16834
/* USER CODE END Header_Chassis_Task_entry */
__weak void Chassis_Task_entry(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task_entry */
  can_filter_init();//can初始�?
  trigger_PID_init();
  shoot_PID_init();
  /* Infinite loop */
  trigger_speed = -2000;
  shoot_speed_1 = 6000;
  shoot_speed_2 = -6000;

  for(;;)
  {
    pid_calc(&trigger_speed_pid,trigger_speed,trigger_info.rotor_speed);
    pid_calc(&shoot_speed_pid_1,shoot_speed_1,shoot_info_1.rotor_speed);
    pid_calc(&shoot_speed_pid_2,shoot_speed_2,shoot_info_2.rotor_speed);
    set_shoot_motor_voltage(&hcan1,trigger_speed_pid.output,shoot_speed_pid_1.output,shoot_speed_pid_2.output);
    osDelay(1);
  }
  /* USER CODE END Chassis_Task_entry */
}

/* USER CODE BEGIN Header_Three_entry */
/**
* @brief Function implementing the Task_three thread.
* @param argument: Not used
* @retval None
*/
#include "dbus_control.h"
/* USER CODE END Header_Three_entry */
__weak void Three_entry(void const * argument)
{
  /* USER CODE BEGIN Three_entry */
  /* Infinite loop */
  // dbus_uart_init();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Three_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
