#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"

#include "CAN_receive.h"
#include "dbus_task.h"

// ���̽ṹ����
#define WHEEL_RADIUS 0.063f                // ��λ��m
#define CHASSIS_RADIUS 0.465f              // ��λ��m
#define REDUCTION_RATIO (3591.0f / 187.0f) // ���ٱ�

// �����ٶ�
#define X_MAX_SPEED 1.0f  // ��λ��m/s
#define Y_MAX_SPEED 1.0f  // ��λ��m/s
#define R_MAX_SPEED 30.0f // ��λ��rpm

// ���ڼ���PID�Ĳ���
#define MOTOR_MAX_CURRENT 2.5f                   // �������������λ��A��3508�����ת����2.5A��CANͨ�����֧��20A����
#define MOTOR_PMAX_ERR (60.0f * REDUCTION_RATIO) // �ﵽ������P�����������λ��rpm��
#define MOTOR_D_RATIO 0.5f

// ����
#define DT 0.001f // �ջ��������ڡ���λ��ms��
#define PI 3.1415926f

typedef struct
{
    float vx; // ���ң���Ϊ�����Σ�
    float vy; // ǰ��ǰΪ������
    float vr; // ƽ��
} chassis_speed_t;

// Ŀ��ֵ����λ��rpm��
fp32 motor1_rpm_set;
fp32 motor2_rpm_set;
fp32 motor3_rpm_set;
fp32 motor4_rpm_set;

// ���ڻ�ȡ���״̬
const motor_measure_t *motor1_status;
const motor_measure_t *motor2_status;
const motor_measure_t *motor3_status;
const motor_measure_t *motor4_status;

// PID�ṹ��
pid_type_def motor1_speed_pid;
pid_type_def motor2_speed_pid;
pid_type_def motor3_speed_pid;
pid_type_def motor4_speed_pid;

void chassis_task_entry(void *argument) {
    // ��ʼ��PID�ṹ��
    const fp32 MOTOR_KP = MOTOR_MAX_CURRENT / MOTOR_PMAX_ERR;
    const fp32 MOTOR_KI = 0;
    const fp32 MOTOR_KD = MOTOR_KP * MOTOR_D_RATIO;
    const fp32 MOTOR_PID_PARA[3] = {MOTOR_KP, MOTOR_KI, MOTOR_KD};
    PID_init(&motor1_speed_pid, PID_POSITION, MOTOR_PID_PARA, MOTOR_MAX_CURRENT, 0);
    PID_init(&motor2_speed_pid, PID_POSITION, MOTOR_PID_PARA, MOTOR_MAX_CURRENT, 0);
    PID_init(&motor3_speed_pid, PID_POSITION, MOTOR_PID_PARA, MOTOR_MAX_CURRENT, 0);
    PID_init(&motor4_speed_pid, PID_POSITION, MOTOR_PID_PARA, MOTOR_MAX_CURRENT, 0);

    // ��ʼ�����ڻ�ȡ���״̬�Ľṹ��
    motor1_status = get_chassis_motor_measure_point(0);
    motor2_status = get_chassis_motor_measure_point(1);
    motor3_status = get_chassis_motor_measure_point(2);
    motor4_status = get_chassis_motor_measure_point(3);

    while (1) {
        if (rc_data.is_connected == 0) { // ң����δ����
            CAN_cmd_chassis(0, 0, 0, 0);
            osDelay(1);
            continue;
        }

        // �˶�ģʽ�л�
        fp32 vx, vy, vr;                  // ��λ��rpm
        if (rc_data.switch_right == UP) { // ǰ���ƶ���������ת
            vx = 0;
            vy = rc_data.y * X_MAX_SPEED / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            vr = rc_data.x * R_MAX_SPEED / (CHASSIS_RADIUS / WHEEL_RADIUS) * 60.0f;
        } else if (rc_data.switch_right == MID || rc_data.switch_right == DOWN) { // ǰ������ƽ��
            vx = rc_data.x * X_MAX_SPEED / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            vy = rc_data.y * Y_MAX_SPEED / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            vr = 0;
        }

        // ȫ�����ٶȺϳ�
        motor1_rpm_set = (-vx - vy + vr) * REDUCTION_RATIO;
        motor2_rpm_set = (vx - vy + vr) * REDUCTION_RATIO;
        motor3_rpm_set = (vx + vy + vr) * REDUCTION_RATIO;
        motor4_rpm_set = (-vx + vy + vr) * REDUCTION_RATIO;

        // ����PID
        PID_calc(&motor1_speed_pid, motor1_status->speed_rpm, motor1_rpm_set);
        PID_calc(&motor2_speed_pid, motor2_status->speed_rpm, motor2_rpm_set);
        PID_calc(&motor3_speed_pid, motor3_status->speed_rpm, motor3_rpm_set);
        PID_calc(&motor4_speed_pid, motor4_status->speed_rpm, motor4_rpm_set);

        // CAN���͵������
        // 3508��������ٱ�3591/187����-20~0~20A => -16384~0~16384
        int16_t motor1_val = motor1_speed_pid.out / 20.0f * 16384.0f;
        int16_t motor2_val = motor2_speed_pid.out / 20.0f * 16384.0f;
        int16_t motor3_val = motor3_speed_pid.out / 20.0f * 16384.0f;
        int16_t motor4_val = motor4_speed_pid.out / 20.0f * 16384.0f;
        CAN_cmd_chassis(motor1_val, motor2_val, motor3_val, motor4_val);

        osDelay(1);
    }
}
