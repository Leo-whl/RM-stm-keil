#include "chassis_task.h"
#include "can.h"
#include "cmsis_os.h"
#include "main.h"
#include <math.h>

#include "dbus_task.h"
#include "pid.h"
#include "struct_typedef.h"

// ���̽ṹ����
#define WHEEL_RADIUS 0.053f   // ���Ӱ뾶����λ��m��
#define CHASSIS_RADIUS 0.428f // ���̰뾶����λ��m��

// �����ٶȲ���
#define VX_MAX 1.0f  // ǰ���˶�����ٶȡ���λ��m/s��
#define VY_MAX 1.0f  // ����ƽ�����λ����λ��m/s��
#define VR_MAX 30.0f // ��ת����ٶȡ���λ��rpm��

// 6020�������
#define M6020_MAX_VOLTAGE 12.0f          // ����ѹ����λ��V����������ѹ24V����
#define M6020_PMAX_ERR 30.0f             // �ﵽ������P�����������λ���Ƕȡ�
#define M6020_STEADY_VOLTAGE_RATIO 0.25f // ��̬��ѹ����
#define M6020_STEADY_ERR 10.0f           // ��̬��PID����Iʱ��������λ���Ƕȡ�
#define M6020_STEADY_T 0.1f              // �ﵽ��̬��ʱ�䳣������λ��s��
#define M6020A_OFFSET 90.0f
#define M6020B_OFFSET 150.0f

// 3508�������
#define M3508_MAX_CURRENT 2.5f // ����������λ��A��3508�����ת����2.5A��CANͨ�����֧��20A����
#define M3508_PMAX_ERR 30.0f   // �ﵽ������P�����������λ�����ת��rpm��
#define M3508_D_RATIO 0.0f
#define M3508_RATIO (3591.0f / 187.0f) // 3508������ٱ�

// ����
#define DT 0.001f // �ջ��������ڡ���λ��ms��
#define PI 3.1415926f

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }
#define ecd_to_angle(ecd) (ecd / 8191.0f * 360.0f) // ������ֵת�Ƕ�

// ���״̬
typedef struct {
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
} motor_status_t;

// �����ٶȷ���
typedef struct {
    fp32 x; // ����λ��rpm����Ч�����ٶȡ�
    fp32 y; // ����λ��rpm����Ч�����ٶȡ�
    fp32 r; // ����λ��rpm����Ч�����ٶȡ�
} v_t;

typedef struct m6020 {
    motor_status_t status;  // ��CAN��ȡ���״̬
    pid_type_def angle_pid; // PID�ṹ��
    fp32 angle_set;         // �Ƕ�Ŀ��ֵ����λ���Ƕȡ�
} m6020_t;

typedef struct m3508 {
    motor_status_t status; // ��CAN��ȡ���״̬
    pid_type_def rpm_pid;  // PID�ṹ��
    fp32 rpm_set;          // �ٶ�Ŀ��ֵ����λ��rpm������ǰ���ٶȡ�
    int8_t dir_set;        // �����ת����1Ϊ��ת��-1Ϊ��ת
} m3508_t;

v_t v;
m6020_t m6020a, m6020b;
m3508_t m3508a, m3508b;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);

    switch (header.StdId) {
    case 0x205: // 6020�����IDΪ1
        get_motor_measure(&m6020a.status, data);
        break;
    case 0x206: // 6020�����IDΪ2
        get_motor_measure(&m6020b.status, data);
        break;
    case 0x207: // 3508�����IDΪ7
        get_motor_measure(&m3508a.status, data);
        break;
    case 0x208: // 3508�����IDΪ8
        get_motor_measure(&m3508b.status, data);
        break;
    }
}

// 6020Ϊ������3508Ϊ�ֵ��
static void can_send_current(fp32 m6020a, fp32 m6020b, fp32 m3508a, fp32 m3508b) {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t send_mail_box;

    // 6020�������ѹ���ƣ�-25000~0~25000����������ѹΪ24V��
    int16_t m6020a_val = m6020a / 24.0f * 25000.0f;
    int16_t m6020b_val = m6020b / 24.0f * 25000.0f;

    // 3508�����C620�������-20~0~20A => -16384~0~16384
    int16_t m3508a_val = m3508a / 20.0f * 16384.0f;
    int16_t m3508b_val = m3508b / 20.0f * 16384.0f;

    header.StdId = 0X1FF;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 0x08;
    data[0] = m6020a_val >> 8; // 6020�����ID��1
    data[1] = m6020a_val;
    data[2] = m6020b_val >> 8; // 6020�����ID��2
    data[3] = m6020b_val;
    data[4] = m3508a_val >> 8; // 3508�����ID��7
    data[5] = m3508a_val;
    data[6] = m3508b_val >> 8; // 3508�����ID��8
    data[7] = m3508b_val;

    HAL_CAN_AddTxMessage(&hcan1, &header, data, &send_mail_box);
}

static float calc_angle_err(float set, float ref) {
    while (set < 0) {
        set += 360.0f;
    }
    while (set > 360) {
        set -= 360.0f;
    }
    while (ref < 0) {
        ref += 360.0f;
    }
    while (ref > 360) {
        ref -= 360.0f;
    }

    float err = fmodf(set - ref + 540.0f, 360.0f) - 180.0f; // ���·�����

    return err;
}

void chassis_task_entry(void *argument) {
    // ��ʼ��PID����
    const fp32 M6020_KP = M6020_MAX_VOLTAGE / M6020_PMAX_ERR;
    const fp32 M6020_KI = M6020_KP * M6020_STEADY_VOLTAGE_RATIO / M6020_STEADY_ERR * DT / M6020_STEADY_T;
    const fp32 M6020_KD = 0;
    PID_init(&m6020a.angle_pid, PID_POSITION, (fp32[3]){M6020_KP, M6020_KI, M6020_KD}, M6020_MAX_VOLTAGE, M6020_MAX_VOLTAGE * M6020_STEADY_VOLTAGE_RATIO);
    PID_init(&m6020b.angle_pid, PID_POSITION, (fp32[3]){M6020_KP, M6020_KI, M6020_KD}, M6020_MAX_VOLTAGE, M6020_MAX_VOLTAGE * M6020_STEADY_VOLTAGE_RATIO);

    const fp32 M3508_KP = M3508_MAX_CURRENT / M3508_PMAX_ERR;
    const fp32 M3508_KI = 0;
    const fp32 M3508_KD = 0;
    PID_init(&m3508a.rpm_pid, PID_POSITION, (fp32[3]){M3508_KP, M3508_KI, M3508_KD}, M3508_MAX_CURRENT, 0);
    PID_init(&m3508b.rpm_pid, PID_POSITION, (fp32[3]){M3508_KP, M3508_KI, M3508_KD}, M3508_MAX_CURRENT, 0);

    while (1) {
        // ���ң��������
        if (rc_data.is_connected == 0) {
            can_send_current(0, 0, 0, 0);
            osDelay(1);
            continue;
        }

        // �����˶�ģʽ����������ٶȷ���
        if (rc_data.switch_right == UP) { // ǰ���ƶ���������ת
            v.x = 0;
            v.y = rc_data.y * VX_MAX / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            v.r = rc_data.x * VR_MAX / (CHASSIS_RADIUS / WHEEL_RADIUS) * 60.0f;
        } else if (rc_data.switch_right == MID || rc_data.switch_right == DOWN) { // ǰ������ƽ��
            v.x = rc_data.x * VX_MAX / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            v.y = rc_data.y * VY_MAX / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            v.r = 0;
        }

        // �����ٶȷ�����������Ŀ��ֵ
        m6020a.angle_set = atan2(v.x, v.y - v.r) / PI * 180.0f + M6020A_OFFSET;
        m6020b.angle_set = atan2(v.x, v.y + v.r) / PI * 180.0f + M6020B_OFFSET;
        m3508a.rpm_set = sqrtf(powf(v.y - v.r, 2) + powf(v.x, 2)) * M3508_RATIO;
        m3508b.rpm_set = -sqrtf(powf(v.y + v.r, 2) + powf(v.x, 2)) * M3508_RATIO;

        // ����PID
        fp32 m6020a_angle_err = calc_angle_err(m6020a.angle_set, ecd_to_angle(m6020a.status.ecd));
        if (m6020a_angle_err < -90.0f) {
            m6020a_angle_err += 180.0f;
            m3508a.dir_set = -1;
        } else if (m6020a_angle_err > 90.0f) {
            m6020a_angle_err -= 180.0f;
            m3508a.dir_set = -1;
        } else {
            m3508a.dir_set = 1;
        }
        PID_calc(&m6020a.angle_pid, 0, m6020a_angle_err);
        PID_calc(&m3508a.rpm_pid, m3508a.status.speed_rpm, m3508a.rpm_set * m3508a.dir_set);

        fp32 m6020b_angle_err = calc_angle_err(m6020b.angle_set, ecd_to_angle(m6020b.status.ecd));
        if (m6020b_angle_err < -90.0f) {
            m6020b_angle_err += 180.0f;
            m3508b.dir_set = -1;
        } else if (m6020b_angle_err > 90.0f) {
            m6020b_angle_err -= 180.0f;
            m3508b.dir_set = -1;
        } else {
            m3508b.dir_set = 1;
        }
        PID_calc(&m6020b.angle_pid, 0, m6020b_angle_err);
        PID_calc(&m3508b.rpm_pid, m3508b.status.speed_rpm, m3508b.rpm_set * m3508b.dir_set);

        // CAN���͵������
        can_send_current(m6020a.angle_pid.out, m6020b.angle_pid.out, m3508a.rpm_pid.out, m3508b.rpm_pid.out);

        osDelay(1);
    }
}
