/*
2024.9.24
WZQ

*/

#include "led.h"

void led_task_entry(void *argument)
{
	while(1)
	{
		HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);		
		osDelay(300);
	}
}











