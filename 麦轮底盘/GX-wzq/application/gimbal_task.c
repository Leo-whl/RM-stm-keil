/*
	2024/11/7
	wzq
*/

#include "gimbal_task.h"

const motor_measure_t *pitch_measure;
const motor_measure_t *yaw_measure;
pid_type_def pitch_pid;
pid_type_def yaw_pid;

const fp32 PID[3]={500.0f,0.0f,20.0f};	//P,I,D
extern fp32 buffer[3];

fp32 set_angle =0;
void gimbal_task(void const * argument)
{
	
	PID_init(&pitch_pid,PID_POSITION,PID,16384,10000);
	PID_init(&yaw_pid,  PID_POSITION,PID,16384,1000);
	
	pitch_measure = get_pitch_gimbal_motor_measure_point();	
	yaw_measure  = get_yaw_gimbal_motor_measure_point();
	
  while(1)
  {			
//			CAN_cmd_gimbal(0,pitch_pid.out,0,0);
	    osDelay(1);
//			PID_calc(&pitch_pid,buffer[0],set_angle);
  }
}


