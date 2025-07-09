#include "chassis.h"
#include "dbus_task.h"

pid_type_def motor_pid0;						//0
const motor_measure_t *motor_data0;
int32_t set_speed0 = 0;						
const fp32 chassis_PID0[3]={5,0,2};
pid_type_def motor_pid1;						//1
const motor_measure_t *motor_data1;
int32_t set_speed1 = 0;						
const fp32 chassis_PID1[3]={5,0,2};	
pid_type_def motor_pid2;						//2
const motor_measure_t *motor_data2;
int32_t set_speed2 = 0;							
const fp32 chassis_PID2[3]={5,0,2};	
pid_type_def motor_pid3;						//3
const motor_measure_t *motor_data3;
int32_t set_speed3 = 0;							
const fp32 chassis_PID3[3]={5,0,2};	
extern move_data cheche_data;
extern uint8_t ctrl_mode;
void loop()
{
	if(ctrl_mode == NO_POWER)
	{
		CAN_cmd_chassis(0,0,0,0);		
	}
	else if(ctrl_mode == REMOTE_CTRL)
	{
		PID_calc(&motor_pid0,motor_data0->speed_rpm, cheche_data.vx+cheche_data.vy);
		PID_calc(&motor_pid1,motor_data1->speed_rpm, cheche_data.vx+cheche_data.vy);	
		PID_calc(&motor_pid2,motor_data2->speed_rpm, cheche_data.vx-cheche_data.vy);
		PID_calc(&motor_pid3,motor_data3->speed_rpm, cheche_data.vx-cheche_data.vy);	
		CAN_cmd_chassis(motor_pid0.out,motor_pid1.out,motor_pid2.out,motor_pid3.out);		
	}
	else if(ctrl_mode == AUTO_CTRL)
	{
	}
}


void chassis_task_entry(void *argument)
{
	can_filter_init();	
	PID_init(&motor_pid0,PID_POSITION,chassis_PID0,16000,2000);	
	PID_init(&motor_pid1,PID_POSITION,chassis_PID1,16000,2000);	
	PID_init(&motor_pid2,PID_POSITION,chassis_PID0,16000,2000);	
	PID_init(&motor_pid3,PID_POSITION,chassis_PID1,16000,2000);	
	motor_data0 = get_chassis_motor_measure_point(0);
	motor_data1 = get_chassis_motor_measure_point(1);
	motor_data2 = get_chassis_motor_measure_point(2);	
	motor_data3 = get_chassis_motor_measure_point(3);
  for(;;)
  {
		loop();
		osDelay(1);
  }
}


