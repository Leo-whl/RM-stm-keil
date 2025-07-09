#include "move.h"
#include "struct_typedef.h"
#include "tim.h"
#include "usart.h"
#include "dbus_task.h"


extern int S2;
extern int servo_mode;
uint16_t Fire_Speed = 0;
int panduan = 0;

void move_task_entry(void const * argument)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	while(1)
	{
	Fire_Speed = 3000;//调整转速
	if(panduan ==2)//右边拨杆拨到中间时开启摩擦轮
	{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Fire_Speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Fire_Speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Fire_Speed);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Fire_Speed);
	}
		osDelay(10);
	}
}
