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
#include "math.h"

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
pid_type_def motor0_pid_err;
pid_type_def motor0_pid_pos;
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
const fp32 PID[3]={0.5,0.05f,0.1};	//P,I,D
const fp32 PID_speed[3] = {60.0,0.0,0.5};
float Fire_Speed = 0;//通过PWM调整射速，0-999
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
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
int aaa = 0;
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
    can_filter_init();
	PID_init(&motor0_pid_err,PID_POSITION,PID,16000,2000);
	PID_init(&motor0_pid_pos,PID_POSITION,PID,100,20);
	PID_init(&motor0_pid,PID_DELTA,PID,16000,2000);
	PID_init(&motor6_pid,PID_DELTA,PID_speed,30000,20000);

	
	motor_data0 = get_chassis_motor_measure_point(0);
	motor_data6 = get_yaw_gimbal_motor_measure_point();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  set_speed = 1000;
  uint32_t pos = 0;
  uint32_t last_pos = 0;
  uint32_t pos_err = 0;
  uint32_t turns = 0;
  uint32_t set_pos = 4000;
  while (1)
  {
//		aaa++;
	  if(motor_data6->ecd<50 ||  motor_data6->ecd>8150 )
	  {
		CAN_cmd_gimbal(0,0,0,0);
		  HAL_Delay(1);
	  }
		  else{
	  set_pos = 3000;
		pos = motor_data6->ecd;
		last_pos = motor_data6->last_ecd;
		pos_err = pos - last_pos;
		PID_calc(&motor0_pid_pos,pos,set_pos);
			  if(set_pos>pos)
			  {
		if(abs(set_pos-pos)>= abs(-pos-8191+set_pos))motor0_pid_pos.out = -motor0_pid_pos.out;//7000-1000    -1000+7000-8191    4000->0  100->8000
		else motor0_pid_pos.out = motor0_pid_pos.out;
			  }
			  else{
		if(abs(set_pos-pos)>= abs(-pos+8191+set_pos))motor0_pid_pos.out = -motor0_pid_pos.out;//7000-1000    -1000+7000-8191    4000->0  100->8000
		else motor0_pid_pos.out = motor0_pid_pos.out;
				  
			  }
		PID_calc(&motor6_pid,motor_data6->speed_rpm,motor0_pid_pos.out);
//		CAN_cmd_chassis(motor0_pid.out,0,0,0);
		CAN_cmd_gimbal(motor6_pid.out,0,0,0);
		HAL_Delay(1);
		  }
		  
//	if( aaa > 5000 && abs(motor_data6->speed_rpm)<100 && 	abs(motor_data6->given_current) > 8000 	) 
//	{
//		
//		for(int i = 0;i<300;i++)
//		{
//			set_speed = -100;//5000
//		PID_calc(&motor6_pid,motor_data6->speed_rpm,set_speed);
////		CAN_cmd_chassis(motor0_pid.out-3000,0,0,0);
//			CAN_cmd_gimbal(0,motor6_pid.out,0,0);
//		HAL_Delay(1);
//		}
//	}
//	else
//	{
//		if(aaa < 5000)
//		{
//		set_speed = 800;
//		PID_calc(&motor6_pid,motor_data6->speed_rpm,set_speed);
////		CAN_cmd_chassis(motor0_pid.out+1000,0,0,0);
//			CAN_cmd_gimbal(0,motor6_pid.out,0,0);
//		HAL_Delay(1);
//		}
//		else
//		{
//		set_speed = 800;//10000
//		PID_calc(&motor6_pid,motor_data6->speed_rpm,set_speed);
////		CAN_cmd_chassis(motor0_pid.out,0,0,0);
//			CAN_cmd_gimbal(0,motor6_pid.out,0,0);
//		HAL_Delay(1);
//		}
//	}
//		
		
		
		

//		HAL_Delay(1);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
