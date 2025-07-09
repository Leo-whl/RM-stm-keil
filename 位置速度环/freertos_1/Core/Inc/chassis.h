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

void CHASSIS_TASK_ENTRY(void *argument);
void rotate_1(int32_t angal);
void angal_set(int16_t __angal);
	
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
