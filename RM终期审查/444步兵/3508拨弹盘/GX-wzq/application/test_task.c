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

pid_type_def motor_pid6;
const motor_measure_t *motor_data6;
const fp32 gimbal_PID6[3]={5,0,2};


pid_type_def motor_pid5;
pid_type_def position_pid5;
pid_type_def speed_pid5;
pid_type_def motor_pid4;
pid_type_def position_pid4;
pid_type_def speed_pid4;
const motor_measure_t *motor_data5;
const motor_measure_t *motor_data4;
motor_measure_t *speed_Data5;
motor_measure_t *speed_Data4;
int set_speed = 0;							//目标速度
const fp32 PID[3]={5,0,0};	//P,I,D
const fp32 __PID[3]={0.5,0,0};	//P,I,D
const fp32 PID__SPEED[3]={5,0.01f,0};
uint32_t last_ecd4=0;
uint32_t last_ecd5=0;
int32_t err5 = 0;
int32_t err4 = 0;
uint32_t turns5=0;
uint32_t turns4=0;
int16_t angal4 = 0;
int16_t angal5 = 0;
int16_t last_angal4=0;
int16_t last_angal5=0;

void angal_set1(int16_t __angal)
{
	if(__angal != last_angal4)
	{
		angal4 += __angal - angal4;	
		rotate_1(angal4-last_angal4);
		last_angal4 = angal4;			
	}
}
void angal_set2(int16_t __angal)
{
	if(__angal != last_angal5)
	{
		angal5 += __angal - angal5;	
		rotate_1(angal5-last_angal5);
		last_angal5 = angal5;			
	}
}

void rotate_1(int32_t __angal)
{
	err4 = ((int32_t)__angal*22.75f);//((8190.0f/360.0f)*19.0f) = 432.25
//	err6 = ((int32_t)__angal*792.0f); //8191/360 = 22.75
}
void rotate_2(int32_t __angal)
{
	err5 = ((int32_t)__angal*22.75f);//((8190.0f/360.0f)*19.0f) = 432.25
//	err6 = ((int32_t)__angal*792.0f); //8191/360 = 22.75
}

void get_rpm1()
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
void get_rpm2()
{
	int32_t realecd5 = motor_data5->ecd;
	if (motor_data5->ecd - motor_data5->last_ecd < -6000)
	{
		turns5 +=1;
		realecd5 += 8192; 
	}
	if (motor_data5->ecd - motor_data5->last_ecd > 6000)
	{
		turns5 -=1;
		realecd5 -= 8192; 
	}
	err5 +=  realecd5 - motor_data5->last_ecd ;
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
	PID_init(&motor_pid4,PID_POSITION,PID,16000,2000);
	PID_init(&motor_pid5,PID_POSITION,PID,16000,2000);
	PID_init(&position_pid4,PID_POSITION,__PID,set_position_speed,500);
	PID_init(&position_pid5,PID_POSITION,__PID,set_position_speed,500);
	PID_init(&speed_pid4,PID_POSITION,PID__SPEED,2000,1000);
	PID_init(&speed_pid5,PID_POSITION,PID__SPEED,2000,1000);
	
	PID_init(&motor_pid6,PID_POSITION,gimbal_PID6,16000,2000);	
	motor_data6 = get_trigger_motor_measure_point();
	
	motor_data5 = get_pitch_gimbal_motor_measure_point();//id为6是PITCH轴6020，
	motor_data4 = get_yaw_gimbal_motor_measure_point();//id为5是yaw轴6020
	last_ecd4 = motor_data4->ecd;
	last_ecd5 = motor_data5->ecd;
    while(1)
    {
			get_rpm1();
			get_rpm2();
			angal_set1(angal4);
			angal_set2(angal5);
			PID_calc(&position_pid4,err4,0);
			PID_calc(&position_pid5,err5,0);
			PID_calc(&motor_pid4,motor_data4->speed_rpm,position_pid4.out);
			PID_calc(&motor_pid5,motor_data5->speed_rpm,position_pid5.out);
		
		    PID_calc(&motor_pid6,motor_data6->speed_rpm,set_speed);
		
			CAN_cmd_gimbal(motor_pid4.out,motor_pid5.out,set_speed,1000);
//			CAN_cmd_gimbal(motor_pid5.out,motor_pid4.out,set_speed,1000);
		osDelay(1);
    }
}



