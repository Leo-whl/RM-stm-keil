#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"


#define DEAD_ILNE 300

#define MAX_SPEED 16000
void chassis_task_entry(void *argument);
void rotate_1(int32_t __angal,uint8_t num);
void angal0_set(int32_t __angal);
void angal1_set(int32_t __angal);
	
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
