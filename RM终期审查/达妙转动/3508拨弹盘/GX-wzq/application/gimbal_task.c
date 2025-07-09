/*
	2024/11/7
	wzq
*/

#include "gimbal_task.h"
#include "dbus_task.h"
/*
控制思路float DM8009_READ_data_1[8]这个数组通过->
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)得到数据；
				DM8009_READ_data_1[3]位置
				DM8009_READ_data_1[4]速度
				DM8009_READ_data_1[5]力
					
   get_motor_inside_pid(moto_measure_t *motor, uint8_t *Data)
   
使能函数	int DM8009_enable_motor(uint16_t id)；return HAL_OK/HAL_ERROR;
失能   void DM8009_stop_motor(uint16_t id)；
保存零点 void DM8009_Save_position(uint16_t id);
发送控制指令  void DM8009_ctrl_motor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos位置, float _vel速度, float _KP, float _KD, float _torq力矩)



*/

//moto_measure_t dm_motor_measure0;

//uint16_t DM_PID0[8] = {0,0,0,0,0,0,0,0};//第3、4位angal p、i；5、6位speed p、i；7、8位转矩p、i
float pitch_rad = 0.0f;//-0.4——0.4
void gimbal_task(void const * argument)
{
		DM8009_Save_position(0x01);
		DM8009_enable_motor(0x01);
	while(1)
	{

		DM8009_ctrl_motor(&hcan2,0x01, pitch_rad, 0.0f, 10.0f, 0.1f, 0.0f);//一个达妙单独插can2
		osDelay(1);


	}
}


