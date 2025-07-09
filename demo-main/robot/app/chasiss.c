/*
2024.9.24
WZQ

*/

#include "chasiss.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


pid_type_def motor_pid[4];
const motor_measure_t *motor_data0;
const motor_measure_t *motor_data1;
const motor_measure_t *motor_data2;
const motor_measure_t *motor_data3;

int16_t wheel_speed[4]= {0,0,0,0};		//目标速度
const fp32 PID[3]={5,0.01f,0};	//P,I,D

void chasiss_task_entry(void *argument)
{
	for(uint8_t i=0;i<4;i++)
	{
		PID_init(&motor_pid[i],PID_POSITION,PID,16000,2000);
	}
	motor_data0 = get_chassis_motor_measure_point(0);
	motor_data1 = get_chassis_motor_measure_point(1);
	motor_data2 = get_chassis_motor_measure_point(2);
	motor_data3 = get_chassis_motor_measure_point(3);

	while(1)
	{
		CAN_cmd_chassis(motor_pid[0].out,motor_pid[1].out,motor_pid[2].out,motor_pid[3].out);
		osDelay(2);
		PID_calc(&motor_pid[0],motor_data0->speed_rpm,wheel_speed[0]);
		PID_calc(&motor_pid[1],motor_data1->speed_rpm,wheel_speed[1]);
		PID_calc(&motor_pid[2],motor_data2->speed_rpm,wheel_speed[2]);
		PID_calc(&motor_pid[3],motor_data3->speed_rpm,wheel_speed[3]);		
		
	}
}











