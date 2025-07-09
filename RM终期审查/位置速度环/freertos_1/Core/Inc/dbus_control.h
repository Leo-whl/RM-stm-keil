#ifndef __DBUS_CONTROL_H_
#define __DBUS_CONTROL_H_

#include "main.h"
#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"
#include "struct_typedef.h"

#define DEADLINE 10
#define UPLINE   660.0f
#define Y_MAX_SPEED 6000.0f
#define X_MAX_SPEED 6000.0f

typedef struct
{
		int16_t vx;
		int16_t vy;
		int16_t vz;
}move_data;

void remote_task_entry(void *argument);

#endif
