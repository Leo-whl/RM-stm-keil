#include "test_task.h"
#include "dbus_task.h"

extern move_data cheche_data;

const motor_measure_t *motor_data4;
const motor_measure_t *motor_data6;

pid_type_def speed_pid;
int8_t shoot_dir;
const fp32 shoot_max_rpm = 60.0f * 3 * 19 / 360 * 60 * 6; // Ò»Ãë3·¢

void test_task(void const *argument) {
    fp32 pid[3] = {3.0f / shoot_max_rpm * 2, 0, 0};
    PID_init(&speed_pid, PID_POSITION, pid, 3.0f, 0);
    motor_data6 = get_trigger_motor_measure_point();
    // motor_data4 = get_yaw_gimbal_motor_measure_point();
    while (1) {
        PID_calc(&speed_pid, motor_data6->speed_rpm, cheche_data.vz * shoot_max_rpm);
        int16_t val = speed_pid.out / 20.0f * 16384.0f;
        CAN_cmd_gimbal(0, 0, val, 0);
        osDelay(1);
    }
}
