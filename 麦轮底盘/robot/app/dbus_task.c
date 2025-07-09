/*
2024.9.24
WZQ

*/

#include "dbus_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control.h"
//#include "bmi088.h"
//#include "math.h"

#define is_up    1
#define is_mid   2
#define is_down  3
#define DEADLINE 10
#define UPLINE   660
#define MAX_SPEED 5000
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
extern int16_t wheel_speed[4];
move_data cheche_data;

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
	
	cheche_data.vx *= MAX_SPEED/UPLINE;
	cheche_data.vy *= MAX_SPEED/UPLINE;	
	cheche_data.vz *= 8000/UPLINE;	
	
	uint8_t S1 = get_switch()%4;
	uint8_t S2 = get_switch()/4;
	
	if(S2 == is_down)		
	{
		cheche_data.vx = 0;
		cheche_data.vy = 0;
		cheche_data.vz = 0;
	}

	
}
void dbus_task_entry(void *argument)
{

	while(1)
	{
		xyz_get();
//extern float yaw_du;
//	float _x = cheche_data.vy*cos(yaw_du)-cheche_data.vx*sin(yaw_du);
//	float _y = cheche_data.vy*sin(yaw_du)+cheche_data.vx*cos(yaw_du);
//		
//	wheel_speed[0] = -_x - _y +  cheche_data.vz;
//    wheel_speed[1] =  _x - _y +  cheche_data.vz;
//    wheel_speed[2] =  _x + _y +  cheche_data.vz;
//    wheel_speed[3] = -_x + _y +  cheche_data.vz;
				
	wheel_speed[0] = -cheche_data.vx - cheche_data.vy +  cheche_data.vz;//一般从右前轮开始逆时针设置id1234
    wheel_speed[1] =  cheche_data.vx - cheche_data.vy +  cheche_data.vz;
    wheel_speed[2] =  cheche_data.vx + cheche_data.vy +  cheche_data.vz;
    wheel_speed[3] = -cheche_data.vx + cheche_data.vy +  cheche_data.vz;		
		osDelay(5);
	}
}











