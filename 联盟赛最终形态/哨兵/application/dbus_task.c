#include "dbus_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "usart.h"

// 摇杆范围
#define RC_XY_MIN 10  // 遥控器摇杆最小值，低于此值输出0
#define RC_XY_MAX 660 // 遥控器摇杆最大值，超过此值钳位

extern sbus_ch_t rc_ctrl;
rc_data_t rc_data;

// 遥杆值归一化到 -1~0~1
static float rc_xy_norm(int16_t val) {
    // 原始值范围：min~mid~max => 364~1024~1684
    val -= 1024;
    if (val > RC_XY_MAX)
        return 1;
    else if (val < -RC_XY_MAX)
        return -1;
    else if (val > -RC_XY_MIN && val < RC_XY_MIN)
        return 0;
    else
        return (float)val / RC_XY_MAX;
}

// 获取拨杆值
static switch_t get_switch(int val) {
    if (val < 750) {
        return DOWN;
    } else if (val < 1250) {
        return MID;
    } else {
        return UP;
    }
}

void dbus_task_entry(void *argument) {
    while (1) {
        rc_data.is_connected = rc_ctrl.ConnectState;
        rc_data.x = rc_xy_norm(rc_ctrl.CH1);
        rc_data.y = rc_xy_norm(rc_ctrl.CH2);
        rc_data.pitch = rc_xy_norm(rc_ctrl.CH3);
        rc_data.yaw = rc_xy_norm(rc_ctrl.CH4);
        rc_data.switch_left = get_switch(rc_ctrl.CH6);  // 左拨杆 CH6
        rc_data.switch_right = get_switch(rc_ctrl.CH7); // 右拨杆 CH7
        osDelay(5);
    }
}
