#include "dbus_task.h"
#include "usart.h"
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
		
extern sbus_ch_t rc_ctrl;
move_data cheche_data;
uint8_t ctrl_mode;
uint8_t servo_mode;

		
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
void xyz_get()
{	
	rc_deadband_limit(rc_ctrl.CH1-1024,cheche_data.vx, DEADLINE, UPLINE);
	rc_deadband_limit(rc_ctrl.CH2-1024,cheche_data.vy, DEADLINE, UPLINE);	
	rc_deadband_limit(rc_ctrl.CH4-1024,cheche_data.vz, DEADLINE, UPLINE);	
	

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
	}
	else if(S2 == is_up)
	{

		ctrl_mode = AUTO_CTRL;
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











