#ifndef DBUS_TASK_H
#define DBUS_TASK_H

#include "struct_typedef.h"

typedef enum {
    DOWN,
    MID,
    UP
} switch_t;

typedef struct {
    uint8_t is_connected;
    float x;
    float y;
    float pitch;
    float yaw;
    switch_t switch_left;
    switch_t switch_right;
} rc_data_t;

extern rc_data_t rc_data;

void dbus_task_entry(void *argument);

#endif // DBUS_TASK_H
