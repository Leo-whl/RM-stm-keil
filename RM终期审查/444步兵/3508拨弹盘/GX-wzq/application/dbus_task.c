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
extern int16_t angal5;
extern sbus_ch_t rc_ctrl;
extern int16_t angal4;
extern int set_speed;
	float pitch_du;
	float yaw_du;
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
	int16_t pitch_zhi;
	rc_deadband_limit(rc_ctrl.CH1-1024,cheche_data.vx, DEADLINE, UPLINE);
	rc_deadband_limit(rc_ctrl.CH2-1024,cheche_data.vy, DEADLINE, UPLINE);	
	rc_deadband_limit(rc_ctrl.CH4-1024,cheche_data.vz, DEADLINE, UPLINE);	
	rc_deadband_limit(rc_ctrl.CH3-1024,pitch_zhi, DEADLINE, UPLINE);

	pitch_du += (float)pitch_zhi/660.0f*0.01f;
	
	if(pitch_du > 25.0f)pitch_du= 25.0f;
	if(pitch_du< -25.0f)pitch_du= -25.0f;
		angal5 =(int16_t)pitch_du;
	yaw_du += (float)cheche_data.vz/660.0f*0.1f;
	if(yaw_du > 180.0f)yaw_du = 180.0f;
	if(yaw_du < -180.0f)yaw_du = -180.0f;
		angal4 = (int16_t)yaw_du;
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
		ctrl_mode = REMOTE_CTRL;
		panduan = 2;
	
	}
	else if(S2 == is_up)
	{
		ctrl_mode = AUTO_CTRL;	
		panduan =1;
	}

	if((ctrl_mode != AUTO_CTRL) && (S1 == is_down))
	{
		servo_mode=0;
		set_speed = 0;
		
	}
	else if((ctrl_mode != AUTO_CTRL) && (S1 == is_mid))
	{
		servo_mode=1;
		set_speed = 1000;
	}	
	else if((ctrl_mode != AUTO_CTRL) && (S1 == is_up))
	{
		servo_mode=2;
		set_speed = 3000;
	}

}

void dbus_task_entry(void *argument)
{

	while(1)
	{
		xyz_get();		
		osDelay(5);		
	}
}
