#ifndef __bmi088_H_
#define __bmi088_H_
#include "main.h"
typedef struct
{
float yaw;
float pitch;
float roll;
}bmi_value;

void bmi_task_entry(void *argument);





#endif
