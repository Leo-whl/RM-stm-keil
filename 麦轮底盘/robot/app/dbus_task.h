#ifndef __DBUS_TASK_H
#define __DBUS_TASK_H
#include "struct_typedef.h"
#include "main.h"


void dbus_task_entry(void *argument);

typedef struct
{
		int16_t vx;
		int16_t vy;
		int16_t vz;
}move_data;

#endif
