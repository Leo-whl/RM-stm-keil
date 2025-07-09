/*
2024.9.19
*/

#include "chassis.h"
#include "dbus_task.h"

pid_type_def motor_pid0;
pid_type_def position_pid0;
const motor_measure_t *motor_data0;
int32_t set_speed0 = 0;							//目标速度
const fp32 chassis_PID0[3]={5,0,2};	//P,I,D
const fp32 __PID0[3]={0.5,0,0};	//P,I,D
uint32_t last_ecd0=0;
int32_t err0 = 0;
uint32_t turns0=0;
int16_t angal0 = 0;
int16_t last_angal0=0;

pid_type_def motor_pid1;
pid_type_def position_pid1;
const motor_measure_t *motor_data1;
int32_t set_speed1 = 0;							//目标速度
const fp32 chassis_PID1[3]={5,0,2};	//P,I,D
const fp32 __PID1[3]={0.5,0,0};	//P,I,D
uint32_t last_ecd1=0;
int32_t err1 = 0;
uint32_t turns1=0;
int16_t angal1 = 0;
int16_t last_angal1=0;

pid_type_def motor_pid2;
pid_type_def motor_pid3;
const motor_measure_t *motor_data2;
int32_t set_speed2 = 0;							//目标速度
const fp32 chassis_PID2[3]={5,0,2};	//P,I,D
const motor_measure_t *motor_data3;
int32_t set_speed3 = 0;							//目标速度
const fp32 chassis_PID3[3]={5,0,2};	//P,I,D

uint8_t flow0_flag = 0;
uint8_t flow1_flag = 0;
uint8_t init_flag  = 1;

extern uint8_t go_flag;
extern move_data cheche_data;
extern uint8_t ctrl_mode;

void angal0_set(int32_t __angal)
{
	if(__angal != last_angal0)
	{
		angal0 += __angal - angal0;	
		rotate_1(angal0-last_angal0,0);  //绝对值角度环
		last_angal0 = angal0;			
	}
}

void angal1_set(int32_t __angal)
{
	if(__angal != last_angal1)
	{
		angal1 += __angal - angal1;	
		rotate_1(angal1-last_angal1,1);
		last_angal1 = angal1;			
	}
}

void rotate_1(int32_t __angal,uint8_t num)
{
	if(num == 0) err0 = ((int32_t)__angal*819.1f);//((8190.0f/360.0f)*36.0f) = 819.1
	else err1 = ((int32_t)__angal*819.1f);//((8190.0f/360.0f)*36.0f) = 819.1
}


void get_rpm()
{
	int32_t realecd0 = motor_data0->ecd;
	if (motor_data0->ecd - motor_data0->last_ecd < -6000)
	{
		turns0 +=1;
		realecd0 += 8192; 
	}
	if (motor_data0->ecd - motor_data0->last_ecd > 6000)
	{
		turns0 -=1;
		realecd0 -= 8192; 
	}
	int32_t realecd1 = motor_data1->ecd;
	if (motor_data1->ecd - motor_data1->last_ecd < -6000)
	{
		turns1 +=1;
		realecd1 += 8192; 
	}
	if (motor_data1->ecd - motor_data1->last_ecd > 6000)
	{
		turns1 -=1;
		realecd1 -= 8192; 
	}	
	
	err0 +=  realecd0 - motor_data0->last_ecd ;
	err1 +=  realecd1 - motor_data1->last_ecd ;
	if(err0 <= -DEAD_ILNE || err0 >= DEAD_ILNE)
	{
		flow0_flag = 1;
	}
	if(err1 <= -DEAD_ILNE || err1 >= DEAD_ILNE)
	{
		flow1_flag = 1;
	}
	if((flow0_flag == 1)&&((err0 >= -DEAD_ILNE && err0 <= DEAD_ILNE)))
	{
		flow0_flag = 0;
		angal0 = 0;
		last_angal0 = 0;
	}
	if((flow1_flag == 1)&&((err1 >= -DEAD_ILNE && err1 <= DEAD_ILNE)))
	{
		flow1_flag = 0;
		angal1 = 0;
		last_angal1 = 0;
	}
	
	if(go_flag == 0)
	{
		flow0_flag = 0;
		angal0 = 0;
		last_angal0 = 0;
		err0 = 0;
		flow1_flag = 0;
		angal1 = 0;
		last_angal1 = 0;
		err1 = 0;
	}

}

extern fp32 buffer[3];
fp32 auto_set_angal = 0.0f;
fp32 angal_err = 0.0f;
void angal_ctrl()
{
	angal_err = auto_set_angal-buffer[0];
}

void loop()
{
//	get_rpm();
//	angal0_set(angal0);
//	angal1_set(angal1);
	if(ctrl_mode == NO_POWER)
	{
		CAN_cmd_chassis(0,0,0,0);		
	}
	else if(ctrl_mode == REMOTE_CTRL)
	{
		PID_calc(&motor_pid0,motor_data0->speed_rpm, -cheche_data.vx-cheche_data.vy);
		PID_calc(&motor_pid1,motor_data1->speed_rpm, cheche_data.vx-cheche_data.vy);	
		PID_calc(&motor_pid2,motor_data2->speed_rpm, cheche_data.vx+cheche_data.vy);
		PID_calc(&motor_pid3,motor_data3->speed_rpm, -cheche_data.vx+cheche_data.vy);	
		CAN_cmd_chassis(motor_pid0.out,motor_pid1.out,motor_pid2.out,motor_pid3.out);		
	}
	else if(ctrl_mode == AUTO_CTRL)
	{
		PID_calc(&motor_pid0,motor_data0->speed_rpm, cheche_data.vy);
		PID_calc(&motor_pid1,motor_data1->speed_rpm, cheche_data.vy);	
		PID_calc(&motor_pid2,motor_data2->speed_rpm, cheche_data.vy);
		PID_calc(&motor_pid3,motor_data3->speed_rpm, cheche_data.vy);	
		CAN_cmd_chassis(motor_pid0.out,motor_pid1.out,motor_pid2.out,motor_pid3.out);		
	}
}


void chassis_task_entry(void *argument)
{
  /* USER CODE BEGIN chassis_task_entry */
	can_filter_init();	
	PID_init(&motor_pid0,PID_POSITION,chassis_PID0,16000,2000);	
	PID_init(&position_pid0,PID_POSITION,__PID0,MAX_SPEED,2000);
	motor_data0 = get_chassis_motor_measure_point(0);
	last_ecd0 = motor_data0->ecd;
	
	PID_init(&motor_pid1,PID_POSITION,chassis_PID1,16000,2000);	
	PID_init(&position_pid1,PID_POSITION,__PID1,MAX_SPEED,2000);
	motor_data1 = get_chassis_motor_measure_point(1);
	last_ecd1 = motor_data1->ecd;
	init_flag = 0;
	
	PID_init(&motor_pid2,PID_POSITION,chassis_PID2,16000,2000);	
	motor_data2 = get_chassis_motor_measure_point(2);	
	PID_init(&motor_pid3,PID_POSITION,chassis_PID3,16000,2000);	
	motor_data3 = get_chassis_motor_measure_point(3);
  /* Infinite loop */
  for(;;)
  {
		loop();
		osDelay(1);
  }
  /* USER CODE END chassis_task_entry */
}


