/*
2024.9.24
WZQ

*/

#include "dbus_task.h"
#include "usart.h"
#include "test_task.h"
#include "gimbal_task.h"
#include "move.h"
#define rc_deadband_limit(input, output, dealine, upline)        																											\
    {                                                    																															\
        if (((input) > (dealine)) || ((input) < -(dealine))) 																													\
        {                                                																															\
            (output) = (input);                          																															\
        }                                                																															\
        else                                            																												  		\
        {                                                																															\
            (output) = 0;                               																															\
        }                                                																															\
				if ((input) > (upline))																																												\
				{																																																							\
						(output) = (upline);																																											\
				}																																																							\
				else if ((input) < (-upline))																																									\
				{																																																							\
						(output) = (-upline);																																											\
				}																																																							\
    }
extern int panduan;
extern float pitch_rad;
extern int16_t angal6;
extern sbus_ch_t rc_ctrl;
extern int16_t angal4;
move_data cheche_data;
uint8_t ctrl_mode;
uint8_t servo_mode;
extern uint8_t auto_servo_mode;
int jishu=1;
		
uint8_t get_switch()
{
	uint8_t result = 0;
	if(rc_ctrl.CH6 == 0)
	{
		result = 0;
	}
	else if (rc_ctrl.CH6 < 750)
	{
		result = is_down;
	}
	else if((rc_ctrl.CH6 > 750) && (rc_ctrl.CH6 < 1250))
	{
		result = is_mid;
	}
	else if(rc_ctrl.CH6 > 1250)
	{
		result = is_up;
	}
	
	if(rc_ctrl.CH7 == 0)
	{
		result = 0;
	}
	else if (rc_ctrl.CH7 < 750)
	{
		result += (is_down << 2);
	}
	else if((rc_ctrl.CH7 > 750) && (rc_ctrl.CH7 < 1250))
	{
		result += (is_mid << 2);
	}
	else if(rc_ctrl.CH7 > 1250)
	{
		result += (is_up << 2);
	}
	return result;
	
}
extern uint16_t auto_set_speed;
extern fp32 auto_set_angal;




uint8_t start[8] = {0x55,0x01,0x02,0x03,0x04,0x05,0x06,0xbb};	
uint8_t end[8]   = {0x55,0x06,0x05,0x04,0x03,0x02,0x01,0xbb};	
void xyz_get()
{	
	int16_t pitch_rad_mid;
	rc_deadband_limit(rc_ctrl.CH1-1024,cheche_data.vx, DEADLINE, UPLINE);
	rc_deadband_limit(rc_ctrl.CH2-1024,cheche_data.vy, DEADLINE, UPLINE);	
	rc_deadband_limit(rc_ctrl.CH4-1024,cheche_data.vz, DEADLINE, UPLINE);	
	rc_deadband_limit(rc_ctrl.CH3-1024,pitch_rad_mid, DEADLINE, UPLINE);
	
	//-660——660->-0.4——0.4
	pitch_rad += (float)pitch_rad_mid/660.0f*0.01f;
	if(pitch_rad > 0.4f)pitch_rad = 0.4f;
	if(pitch_rad < -0.4f)pitch_rad = -0.4f;
	angal4 += cheche_data.vz/660*1;
	if(angal4 > 180)angal4 = 180;
	if(angal4 < -180)angal4 = -180;
	cheche_data.vx *= (X_MAX_SPEED/UPLINE);
	cheche_data.vy *= (Y_MAX_SPEED/UPLINE);	
	cheche_data.vz *= 8000/UPLINE;	
	
	uint8_t S1 = get_switch()%4;
	uint8_t S2 = get_switch()/4;
	
	if((S2 == is_down) || (rc_ctrl.ConnectState == 0))		
	{
		ctrl_mode = NO_POWER;

		cheche_data.vx = 0;
		cheche_data.vy = 0;	
		cheche_data.vz = 0;	
	}
	else if(S2 == is_mid)
	{
		if(ctrl_mode == AUTO_CTRL)
		{
			HAL_UART_Transmit_IT(&huart1,start,8);
		}
		ctrl_mode = REMOTE_CTRL;
		panduan = 0;
	
	}
	else if(S2 == is_up)
	{
		if(ctrl_mode == REMOTE_CTRL)
		{
			HAL_UART_Transmit_IT(&huart1,end,8);

		}
		ctrl_mode = AUTO_CTRL;	
		panduan =1;
	}

	if(ctrl_mode == AUTO_CTRL)
	{
		servo_mode = auto_servo_mode;
	}
	if((ctrl_mode != AUTO_CTRL) && (S1 == is_down))
	{
		servo_mode=0;
		if(jishu == 0)
		{
		angal6-=185;
		jishu=1;
		}
		
	}
	else if((ctrl_mode != AUTO_CTRL) && (S1 == is_mid))
	{
		servo_mode=1;
		jishu = 0;
		
	}	
	else if((ctrl_mode != AUTO_CTRL) && (S1 == is_up))
	{
		servo_mode=2;
		if(jishu == 0)
		{
		angal6+=185;
		jishu=1;
		}
	}

}

void dbus_task_entry(void *argument)
{

	while(1)
	{
		xyz_get();	
extern int32_t set_speed0 ;
extern int32_t set_speed1 ;	
extern int32_t set_speed2 ;
extern int32_t set_speed3 ;		
//	set_speed3 = -cheche_data.vx - cheche_data.vy +  cheche_data.vz;//一般从右前轮开始逆时针设置id1234
//    set_speed0  =  cheche_data.vx - cheche_data.vy +  cheche_data.vz;
//    set_speed1  =  cheche_data.vx + cheche_data.vy +  cheche_data.vz;
//    set_speed2  = -cheche_data.vx + cheche_data.vy +  cheche_data.vz;
		
		osDelay(5);		
	}
}











