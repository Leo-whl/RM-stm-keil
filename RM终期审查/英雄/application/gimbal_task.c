/*
	2024/11/7
	wzq
*/

#include "gimbal_task.h"
#include "dbus_task.h"
/*
����˼·float DM8009_READ_data_1[8]�������ͨ��->
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)�õ����ݣ�
				DM8009_READ_data_1[3]λ��
				DM8009_READ_data_1[4]�ٶ�
				DM8009_READ_data_1[5]��
					
   get_motor_inside_pid(moto_measure_t *motor, uint8_t *Data)
   
ʹ�ܺ���	int DM8009_enable_motor(uint16_t id)��return HAL_OK/HAL_ERROR;
ʧ��   void DM8009_stop_motor(uint16_t id)��
������� void DM8009_Save_position(uint16_t id);
���Ϳ���ָ��  void DM8009_ctrl_motor(CAN_HandleTypeDef* hcan,uint16_t id, float _posλ��, float _vel�ٶ�, float _KP, float _KD, float _torq����)



*/

//moto_measure_t dm_motor_measure0;

//uint16_t DM_PID0[8] = {0,0,0,0,0,0,0,0};//��3��4λangal p��i��5��6λspeed p��i��7��8λת��p��i
float pitch_rad = 0.0f;//-0.4����0.4
void gimbal_task(void const * argument)
{
		DM8009_Save_position(0x01);
		DM8009_enable_motor(0x01);
	while(1)
	{

		DM8009_ctrl_motor(&hcan2,0x01, pitch_rad, 0.0f, 10.0f, 0.1f, 0.0f);//һ���������can2
		osDelay(1);


	}
}


