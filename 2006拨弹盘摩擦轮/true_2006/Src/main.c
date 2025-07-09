/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "pid.h"
#include "remote_control.h"

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
pid_type_def motor0_pid;
pid_type_def motor1_pid;
pid_type_def motor2_pid;
pid_type_def motor3_pid;
pid_type_def motor4_pid;
pid_type_def motor5_pid;
pid_type_def motor6_pid;
pid_type_def motor7_pid;

const RC_ctrl_t *local_rc_ctrl;

const motor_measure_t *motor_data0;	//声明电机结构体
const motor_measure_t *motor_data1;	//声明电机结构体
const motor_measure_t *motor_data2;	//声明电机结构体
const motor_measure_t *motor_data3;	//声明电机结构体
const motor_measure_t *motor_data4;	//声明电机结构体
const motor_measure_t *motor_data5;	//声明电机结构体
const motor_measure_t *motor_data6;	//声明电机结构体
const motor_measure_t *motor_data7;
int set_speed = 0;							//目标速度
const fp32 PID[3]={5,0.01f,0};	//P,I,D
float Fire_Speed = 0;//通过PWM调整射速，0-999
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);

  
    can_filter_init();
	PID_init(&motor0_pid,PID_DELTA,PID,16000,2000);
	PID_init(&motor1_pid,PID_DELTA,PID,16000,2000);
	PID_init(&motor2_pid,PID_POSITION,PID,16000,2000);
	PID_init(&motor3_pid,PID_POSITION,PID,16000,2000);
	PID_init(&motor4_pid,PID_POSITION,PID,16000,2000);
	PID_init(&motor5_pid,PID_POSITION,PID,16000,2000);
	PID_init(&motor6_pid,PID_POSITION,PID,1000,2000);
	PID_init(&motor7_pid,PID_POSITION,PID,16000,2000);
	
	motor_data0 = get_chassis_motor_measure_point(0);
	motor_data1 = get_chassis_motor_measure_point(1);
	motor_data2 = get_chassis_motor_measure_point(2);
	motor_data3 = get_chassis_motor_measure_point(3);
	motor_data4 = get_chassis_motor_measure_point(4);
	motor_data5 = get_chassis_motor_measure_point(5);
	motor_data6 = get_trigger_motor_measure_point();
	motor_data7 = get_chassis_motor_measure_point(7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
	  Fire_Speed = 250.0f-90.0f*(200.0f/180.0f);//200000--------20000
	  Fire_Speed = 250.0f;//200000--------
	  
	  Fire_Speed = 50.0f;//200000--------20000
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Fire_Speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Fire_Speed);//调整开火速度
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Fire_Speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Fire_Speed);//调整开火速度
	  
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,Fire_Speed);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,Fire_Speed);//调整开火速度
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,Fire_Speed);

	

	
	  
      set_speed=1000;		  
				CAN_cmd_chassis(motor0_pid.out, motor1_pid.out, motor2_pid.out, motor3_pid.out);
//				CAN_cmd_gimbal(motor4_pid.out, motor5_pid.out, motor6_pid.out, motor7_pid.out);
				HAL_Delay(10);
	  
		PID_calc(&motor0_pid,motor_data0->speed_rpm,set_speed);
		PID_calc(&motor1_pid,motor_data1->speed_rpm,set_speed);
		PID_calc(&motor2_pid,motor_data2->speed_rpm,set_speed);
		PID_calc(&motor3_pid,motor_data3->speed_rpm,set_speed);
		PID_calc(&motor6_pid,motor_data6->speed_rpm,set_speed);
	 

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
