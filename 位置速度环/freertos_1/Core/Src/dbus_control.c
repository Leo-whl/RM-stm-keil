#include "dbus_control.h"
#include "bsp_rc.h"
#include "usart.h"
#include "chassis.h"

#define rc_deadband_limit(input, output, dealine, upline)     																											\
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
    }//遥控器数据获得
	
extern sbus_ch_t rc_ctrl;//遥控器数据储存处
move_data speed_data;
extern int16_t angal;
int zanshi = 0;

uint8_t get_switch()//俩拨杆挡位获得1:最下面，2：中间，3：最上面
{
	uint8_t result = 0;
	if(rc_ctrl.CH6 == 0)
	{
		result = 0;
	}
	else if (rc_ctrl.CH6 < 750)
	{
		result = 1;
	}
	else if((rc_ctrl.CH6 > 750) && (rc_ctrl.CH6 < 1250))
	{
		result = 2;
	}
	else if(rc_ctrl.CH6 > 1250)
	{
		result = 3;
	}
	
	if(rc_ctrl.CH7 == 0)
	{
		result = 0;
	}
	else if (rc_ctrl.CH7 < 750)
	{
		result += (1 << 2);
	}
	else if((rc_ctrl.CH7 > 750) && (rc_ctrl.CH7 < 1250))
	{
		result += (2 << 2);
	}
	else if(rc_ctrl.CH7 > 1250)
	{
		result += (3 << 2);
	}
	return result;
}


void xyz_get()//速度获得
{	
	rc_deadband_limit(rc_ctrl.CH1-1024,speed_data.vx, DEADLINE, UPLINE);
	rc_deadband_limit(rc_ctrl.CH2-1024,speed_data.vy, DEADLINE, UPLINE);	
	rc_deadband_limit(rc_ctrl.CH4-1024,speed_data.vz, DEADLINE, UPLINE);
	uint8_t S1 = get_switch()%4;
	uint8_t S2 = get_switch()/4;
	
	if((S2 == 1) || (rc_ctrl.ConnectState == 0))		
	{
		
		
		speed_data.vx = 0;
		speed_data.vy = 0;	
		speed_data.vz = 0;	
	}
	else if(S2 == 2)
	{
	angal+=185;
	}
	else if(S2 == 3)
	{
		
	}
	
	speed_data.vx *= (X_MAX_SPEED/UPLINE);
	speed_data.vy *= (Y_MAX_SPEED/UPLINE);	
	speed_data.vz *= 8000/UPLINE;	
}


void remote_task_entry(void * argument)
{

	for(;;)
	{
		xyz_get();
		osDelay(5);
	}
}
	
