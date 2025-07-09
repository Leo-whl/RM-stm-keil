/*
2024.9.19
*/

#include "chassis.h"
int speed = 0;
pid_type_def motor_pid;
pid_type_def position_pid;
pid_type_def speed_pid;
const motor_measure_t *motor_data;
motor_measure_t *speed_Data;
int set_speed = 0;							//目标速度
const fp32 PID[3]={5,0,0};	//P,I,D
const fp32 __PID[3]={0.5,0,0};	//P,I,D
const fp32 PID__SPEED[3]={5,0.01f,0};
uint32_t last_ecd=0;

int32_t err = 0;
uint32_t turns=0;
int16_t angal = 0;
int16_t last_angal=0;

void angal_set(int16_t __angal)
{
	if(__angal != last_angal)
	{
		angal += __angal - angal;	
		rotate_1(angal-last_angal);
		last_angal = angal;			
	}
}

void rotate_1(int32_t __angal)
{
	err = ((int32_t)__angal*517.5625f);
//	err = ((int32_t)__angal*432.25f);//((8190.0f/360.0f)*19.0f) = 432.25//3508
//	err = ((int32_t)__angal*792.0f);//((8190.0f/360.0f)*36.0f) = 792	//2006
}

void get_rpm()
{
	int32_t realecd = motor_data->ecd;
	if (motor_data->ecd - motor_data->last_ecd < -6000)
	{
		turns +=1;
		realecd += 8192; 
	}
	if (motor_data->ecd - motor_data->last_ecd > 6000)
	{
		turns -=1;
		realecd -= 8192; 
	}
	err +=  realecd - motor_data->last_ecd ;

}

void CHASSIS_TASK_ENTRY(void *argument)
{
  /* USER CODE BEGIN chassis_task_entry */
	can_filter_init();	
	PID_init(&motor_pid,PID_DELTA,PID,16000,2000);
	PID_init(&position_pid,PID_POSITION,__PID,3000,2000);
	PID_init(&speed_pid,PID_POSITION,PID__SPEED,16000,2000);
	motor_data = get_chassis_motor_measure_point(0);
	last_ecd = motor_data->ecd;
	
  /* Infinite loop */
  for(;;)
  {
		get_rpm();
		angal_set(angal);
	  set_speed = 8000;
		PID_calc(&position_pid,err,0);
	  PID_calc(&motor_pid,motor_data->speed_rpm,position_pid.out);
		PID_calc(&speed_pid,motor_data->speed_rpm,set_speed);
	  CAN_cmd_chassis(motor_pid.out,3000,set_speed,1000);
	  CAN_cmd_gimbal(motor_pid.out,0,0,0);
		osDelay(1);

  }
  /* USER CODE END chassis_task_entry */
}


