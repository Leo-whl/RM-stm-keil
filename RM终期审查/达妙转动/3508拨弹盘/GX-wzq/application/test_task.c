/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
int set_position_speed =1000;
int speed = 0;
pid_type_def motor_pid4;
pid_type_def position_pid4;
pid_type_def speed_pid4;
pid_type_def motor_pid6;
pid_type_def position_pid6;
pid_type_def speed_pid6;
const motor_measure_t *motor_data4;
const motor_measure_t *motor_data6;
motor_measure_t *speed_Data4;
motor_measure_t *speed_Data6;
int set_speed = 0;							//目标速度
const fp32 PID[3]={5,0,0};	//P,I,D
const fp32 __PID[3]={0.5,0,0};	//P,I,D
const fp32 PID__SPEED[3]={5,0.01f,0};
uint32_t last_ecd6=0;
uint32_t last_ecd4=0;
int32_t err4 = 0;
int32_t err6 = 0;
uint32_t turns4=0;
uint32_t turns6=0;
int16_t angal6 = 0;
int16_t angal4 = 0;
int16_t last_angal6=0;
int16_t last_angal4=0;

void angal_set1(int16_t __angal)
{
	if(__angal != last_angal6)
	{
		angal6 += __angal - angal6;	
		rotate_1(angal6-last_angal6);
		last_angal6 = angal6;			
	}
}
void angal_set2(int16_t __angal)
{
	if(__angal != last_angal4)
	{
		angal4 += __angal - angal4;	
		rotate_1(angal4-last_angal4);
		last_angal4 = angal4;			
	}
}

void rotate_1(int32_t __angal)
{
	err6 = ((int32_t)__angal*432.25f);//((8190.0f/360.0f)*19.0f) = 432.25
//	err6 = ((int32_t)__angal*792.0f); //8191/360 = 22.75
}
void rotate_2(int32_t __angal)
{
	err4 = ((int32_t)__angal*22.75f);//((8190.0f/360.0f)*19.0f) = 432.25
//	err6 = ((int32_t)__angal*792.0f); //8191/360 = 22.75
}

void get_rpm1()
{
	int32_t realecd6 = motor_data6->ecd;
	if (motor_data6->ecd - motor_data6->last_ecd < -6000)
	{
		turns6 +=1;
		realecd6 += 8192; 
	}
	if (motor_data6->ecd - motor_data6->last_ecd > 6000)
	{
		turns6 -=1;
		realecd6 -= 8192; 
	}
	err6 +=  realecd6 - motor_data6->last_ecd ;

}
void get_rpm2()
{
	int32_t realecd4 = motor_data4->ecd;
	if (motor_data4->ecd - motor_data4->last_ecd < -6000)
	{
		turns4 +=1;
		realecd4 += 8192; 
	}
	if (motor_data4->ecd - motor_data4->last_ecd > 6000)
	{
		turns4 -=1;
		realecd4 -= 8192; 
	}
	err4 +=  realecd4 - motor_data4->last_ecd ;
}


/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
	can_filter_init();	
	PID_init(&motor_pid6,PID_POSITION,PID,16000,2000);
	PID_init(&motor_pid4,PID_POSITION,PID,16000,2000);
	PID_init(&position_pid6,PID_POSITION,__PID,set_position_speed,500);
	PID_init(&position_pid4,PID_POSITION,__PID,set_position_speed,500);
	PID_init(&speed_pid6,PID_POSITION,PID__SPEED,2000,1000);
	PID_init(&speed_pid4,PID_POSITION,PID__SPEED,2000,1000);
	motor_data6 = get_trigger_motor_measure_point();
	motor_data4 = get_yaw_gimbal_motor_measure_point();
	last_ecd6 = motor_data6->ecd;
	last_ecd4 = motor_data4->ecd;
    while(1)
    {
			get_rpm1();
			get_rpm2();
			angal_set1(angal6);
			angal_set2(angal4);
			PID_calc(&position_pid6,err6,0);
			PID_calc(&position_pid4,err4,0);
			PID_calc(&motor_pid6,motor_data6->speed_rpm,position_pid6.out);
			PID_calc(&motor_pid4,motor_data4->speed_rpm,position_pid4.out);
			PID_calc(&speed_pid6,motor_data6->speed_rpm,set_speed);
			PID_calc(&speed_pid4,motor_data4->speed_rpm,set_speed);
			//CAN_cmd_gimbal(motor_pid4.out,3000,motor_pid6.out,1000);
		osDelay(1);
    }
}



