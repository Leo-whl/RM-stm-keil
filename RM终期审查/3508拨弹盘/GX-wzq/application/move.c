#include "move.h"
#include "struct_typedef.h"
#include "tim.h"
#include "usart.h"
#include "dbus_task.h"
extern int16_t angal0;
extern int16_t angal1;
extern fp32 buffer[3];
extern move_data cheche_data;

#define move_delay(time) if(go_flag == 0)return 0;\
														osDelay(time);
uint8_t go_flag = 0;
extern uint8_t servo_mode;

uint8_t ok[8]={0x55,0x01,0x02,0x03,0x04,0x05,0x06,0xbb};	

void return_ok()
{
	HAL_UART_Transmit_IT(&huart1,ok,8);
}


void go(uint32_t mm)
{	
	if(go_flag)
	{
		angal0 =  (mm/0.6f);
		angal1 = -(mm/0.6f);		
		while((angal0 ) || (angal1 ));
		return_ok();
	}
}

void back(uint32_t mm)
{	
	if(go_flag)
	{
		angal0 =  -(mm/0.6f);
		angal1 =   (mm/0.6f);		
		while((angal0 ) || (angal1 ));
		return_ok();		
	}
}


void ctrl_servo(uint8_t angal)
{

	TIM8->CCR3 = angal*(200.0f/180.0f)+45;
	TIM8->CCR2 = 250-angal*(200.0f/180.0f) ;
}

extern uint8_t auto_mode;
extern uint16_t auto_set_speed;
extern int32_t set_speed0 ;
extern int32_t set_speed1 ;
extern int32_t set_speed2 ;
extern int32_t set_speed3 ;
extern uint8_t SBUS_CH;

uint8_t move_task()
{

	if(servo_mode == 0)
	{
		ctrl_servo(0);
	}
	else if(servo_mode == 1)
	{
		ctrl_servo(120);
	}
	else if(servo_mode == 2)
	{
		ctrl_servo(167);
	}
	
	if(auto_mode == 0)
	{
		set_speed0 = -auto_set_speed;
		set_speed1 =  auto_set_speed;
		set_speed2 =  auto_set_speed;
		set_speed3 = -auto_set_speed;
	}
	else if(auto_mode == 1)
	{
		set_speed0 =  auto_set_speed;
		set_speed1 = -auto_set_speed;
		set_speed2 = -auto_set_speed;
		set_speed3 =  auto_set_speed;
	}
}



extern uint8_t init_flag;
void move_task_entry(void const * argument)
{
	while(init_flag);	
	while(1)
	{
		move_task();
//		turn(0);
		osDelay(10);
	}
}
