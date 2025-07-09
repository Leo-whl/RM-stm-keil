/*


*/

#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"

#define PITCH_TURN 1
#define YAW_TURN   1


void gimbal_task(void const * argument);





#endif



