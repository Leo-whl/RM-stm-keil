#include "dbus_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "usart.h"

// ҡ�˷�Χ
#define RC_XY_MIN 10  // ң����ҡ����Сֵ�����ڴ�ֵ���0
#define RC_XY_MAX 660 // ң����ҡ�����ֵ��������ֵǯλ

extern sbus_ch_t rc_ctrl;
rc_data_t rc_data;

// ң��ֵ��һ���� -1~0~1
static float rc_xy_norm(int16_t val) {
    // ԭʼֵ��Χ��min~mid~max => 364~1024~1684
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

// ��ȡ����ֵ
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
        rc_data.switch_left = get_switch(rc_ctrl.CH6);  // �󲦸� CH6
        rc_data.switch_right = get_switch(rc_ctrl.CH7); // �Ҳ��� CH7
        osDelay(5);
    }
}
