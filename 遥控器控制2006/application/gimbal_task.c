#include "gimbal_task.h"
#include "dbus_task.h"
fp32 set_angle =0;
pid_type_def motor_pid6;						//0
const motor_measure_t *motor_data6;
int32_t set_speed6 = 0;						
const fp32 gimbal_PID6[3]={5,0,2};
int set_speed = 0;
void gimbal_task(void const * argument)
{
	PID_init(&motor_pid6,PID_POSITION,gimbal_PID6,16000,8000);
	motor_data6 = get_trigger_motor_measure_point();
	int trigger_flag = 0;
  while(1)
  {			
	  if(S1 == 1)
	  {
		  if(trigger_flag == 0)
		  {
			trigger_flag = 1 ;
		  }
	  }
	  if(S1 == 3)
	  {
		if(trigger_flag == 1)
		{
			set_speed +=1000;
			trigger_flag = 0;
		}
	  }
	  
	  if(S2 == 1)
	  {
		PID_calc(&motor_pid6,motor_data6->speed_rpm,set_speed);
		CAN_cmd_gimbal(0,0,motor_pid6.out,0);
	  }
	  else
	  {
		PID_calc(&motor_pid6,motor_data6->speed_rpm,0);
		CAN_cmd_gimbal(0,0,motor_pid6.out,0);
	  }
	    osDelay(1);
  }
  
}



