#ifndef __DBUS_TASK_H
#define __DBUS_TASK_H
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"


void dbus_task_entry(void *argument);

#define is_up    1
#define is_mid   2
#define is_down  3
#define DEADLINE 10
#define UPLINE   660.0f
#define Y_MAX_SPEED 6000.0f
#define X_MAX_SPEED 6000.0f

static enum
{
	NO_POWER = 0,
	REMOTE_CTRL,
	AUTO_CTRL,	
	
}CTRL_mode;

typedef struct
{
		int16_t vx;
		int16_t vy;
		int16_t vz;
}move_data;
extern uint8_t S1;
extern uint8_t S2;
#endif
