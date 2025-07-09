#include "gimbal_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "struct_typedef.h"
#include "tim.h"
#include <math.h>

#include "CAN_receive.h"
#include "dbus_task.h"

// pitch�������
#define PTICH_MAX_SPEED 100.0f // pitch�������ٶȡ���λ���Ƕ�/s��
#define PITCH_MAX_ANGLE 50.0f  // pitch������Ƕ�
// ���ڼ���PID�Ĳ���
#define PITCH_MAX_CURRENT 3.0f    // ����������λ��A��6020�����ת����0.9A��CANͨ�����֧��3A����
#define PITCH_PMAX_ERR 5.0f       // �ﵽ������P�����������λ���Ƕȡ�
#define PITCH_STEADY_CURRENT 0.6f // ��̬��������λ��A��
#define PITCH_STEADY_ERR 20.0f    // ��̬��PID����Iʱ��������λ���Ƕȡ�
#define PITCH_STEADY_T 0.1f       // �ﵽ��̬��ʱ�䳣������λ��s��
#define PITCH_D_RATIO 0.0f

// yaw�������
#define YAW_MAX_SPEED 360.0f // yaw�������ٶȡ���λ���Ƕ�/s��
// ���ڼ���PID�Ĳ���
#define YAW_MAX_CURRENT 3.0f // ����������λ��A��6020�����ת����0.9A��CANͨ�����֧��3A����
#define YAW_PMAX_ERR 10.0f   // �ﵽ������P�����������λ���Ƕ�
#define YAW_D_RATIO (-0.5f)

// �������
#define SHOOT_FREQ 1 // ���䵯��Ƶ�ʣ�ÿ���Ӷ��ٷ���
// ���ڼ���PID�Ĳ���
#define SHOOT_MAX_CURRENT 5.0f                     // ����������λ��A��6002������10A����
#define SHOOT_SPEED (360.0f / 8 * SHOOT_FREQ * 36) // ��������ٶȡ���λ���Ƕ�/s����8ΪתһȦ����ĵ�������36Ϊ6002������ٱȣ�
#define SHOOT_PMAX_SPEED SHOOT_SPEED               // �ﵽ���ٶȺ�P�����������λ���Ƕ�/s��

// ����
#define DT 0.001f // �ջ��������ڡ���λ��ms��

// Ŀ��ֵ
fp32 yaw_angle_set;   // ��λ���Ƕ�
fp32 pitch_angle_set; // ��λ���Ƕ�
fp32 shoot_speed_set; // ��λ���Ƕ�/s
uint8_t wheel_enable; // Ħ����ʹ��

// �����ʼλ��
fp32 yaw_offset;
fp32 pitch_offset;

// ���ڻ�ȡ���״̬
const motor_measure_t *yaw_motor_status;
const motor_measure_t *pitch_motor_status;
const motor_measure_t *shoot_motor_status;

// PID�ṹ��
pid_type_def yaw_angle_pid;
pid_type_def pitch_angle_pid;
pid_type_def shoot_speed_pid;

static void init_pid() {
    // ������������: {P, I, D}, MAX_OUT, MAX_I

    // yaw��� - λ��PID��PID����Ƕȣ����������
    const fp32 YAW_KP = YAW_MAX_CURRENT / YAW_PMAX_ERR;
    const fp32 YAW_KI = 0;
    const fp32 YAW_KD = YAW_KP * YAW_D_RATIO;
    const fp32 yaw_pid_para[3] = {YAW_KP, YAW_KI, YAW_KD};
    PID_init(&yaw_angle_pid, PID_POSITION, yaw_pid_para, YAW_MAX_CURRENT, 0);

    // pitch��� - λ��PID��PID����Ƕȣ����������
    const fp32 PITCH_KP = PITCH_MAX_CURRENT / PITCH_PMAX_ERR;
    const fp32 PITCH_KI = PITCH_STEADY_CURRENT / PITCH_STEADY_ERR * DT / PITCH_STEADY_T;
    const fp32 PITCH_KD = PITCH_KP * PITCH_D_RATIO;
    const fp32 pitch_pid_para[3] = {PITCH_KP, PITCH_KI, PITCH_KD};
    PID_init(&pitch_angle_pid, PID_POSITION, pitch_pid_para, PITCH_MAX_CURRENT, PITCH_STEADY_CURRENT * 1.2f);

    // ������� - �ٶ�PID��PID�����ٶȣ����������
    const fp32 SHOOT_KP = SHOOT_MAX_CURRENT / SHOOT_PMAX_SPEED;
    const fp32 SHOOT_KI = 0;
    const fp32 SHOOT_KD = 0;
    const fp32 shoot_pid_para[3] = {SHOOT_KP, SHOOT_KI, SHOOT_KD};
    PID_init(&shoot_speed_pid, PID_POSITION, shoot_pid_para, SHOOT_MAX_CURRENT, 0);
}

static void init_motor_status() {
    // ���ڻ�ȡ�������״̬
    yaw_motor_status = get_yaw_gimbal_motor_measure_point();
    pitch_motor_status = get_pitch_gimbal_motor_measure_point();
    shoot_motor_status = get_trigger_motor_measure_point(); // �������

    // ȷ���յ��������
    osDelay(500);

    // ��ȡ�����ʼλ��
    yaw_offset = yaw_motor_status->ecd / 8191.0f * 360.0f;
    pitch_offset = pitch_motor_status->ecd / 8191.0f * 360.0f;
}

static void handle_rc() {
    if (rc_data.is_connected == 0) { // ң����δ����
        wheel_enable = 0;
        shoot_speed_set = 0;
        return;
    }

    // pitch���
    pitch_angle_set += rc_data.pitch * PTICH_MAX_SPEED * DT;
    if (pitch_angle_set > PITCH_MAX_ANGLE)
        pitch_angle_set = PITCH_MAX_ANGLE;
    if (pitch_angle_set < 0)
        pitch_angle_set = 0;

    // yaw���
    yaw_angle_set += (-rc_data.yaw) * YAW_MAX_SPEED * DT;
    if (yaw_angle_set > 360)
        yaw_angle_set -= 360;
    if (yaw_angle_set < 0)
        yaw_angle_set += 360;

    // Ħ���֡�������������󲦸˿���
    if (rc_data.switch_left == UP) { // �رգ�Ħ���ֹء����������
        wheel_enable = 0;
        shoot_speed_set = 0;
    } else if (rc_data.switch_left == MID) { // ׼����Ħ���ֿ������������
        wheel_enable = 1;
        shoot_speed_set = 0;
    } else if (rc_data.switch_left == DOWN) { // ���䣺Ħ���ֿ������������
        wheel_enable = 1;
        shoot_speed_set = SHOOT_SPEED;
    }
}

static void calc_pid() {
    // yaw pid λ�û�
    // ecdΪ���������ֵ��0~8191 ��Ӧ 0~360��
    fp32 yaw_angle_now = yaw_motor_status->ecd / 8191.0f * 360.0f;       // yaw��ǰ�Ƕ�
    fp32 yaw_angle_ex = yaw_angle_set + yaw_offset;                      // yaw�����Ƕ�
    fp32 yaw_err = fmodf(yaw_angle_ex - yaw_angle_now + 180, 360) - 180; // ���·�����
    PID_calc(&yaw_angle_pid, 0, yaw_err);

    // pitch pid λ�û�
    // ecdΪ���������ֵ��0~8191 ��Ӧ 0~360��
    fp32 pitch_angle_now = pitch_motor_status->ecd / 8191.0f * 360.0f;         // pitch��ǰ�Ƕ�
    fp32 pitch_angle_ex = pitch_angle_set + pitch_offset;                      // pitch�����Ƕ�
    fp32 pitch_err = fmodf(pitch_angle_ex - pitch_angle_now + 180, 360) - 180; // ���·�����
    PID_calc(&pitch_angle_pid, 0, pitch_err);

    // shoot pid �ٶȻ�
    // ecdΪ���������ֵ��0~8191 ��Ӧ 0~360��
    fp32 shoot_speed_now = shoot_motor_status->speed_rpm / 60.0f * 360.0f; // shoot�ٶȵ�ǰֵ����λ���Ƕ�/s��
    PID_calc(&shoot_speed_pid, shoot_speed_now, shoot_speed_set);
}

static void motor_control() {
    // 6020�����-3A~0~3A => -16384~0~16384
    // 2006��������ٱ�36����-10~0~10A => -10000~0~10000
    int16_t yaw_val = yaw_angle_pid.out / 3.0f * 16384.0f;
    int16_t pitch_val = pitch_angle_pid.out / 3.0f * 16384.0f;
    int16_t shoot_val = shoot_speed_pid.out / 10.0f * 10000.0f;
    // CAN����yaw pitch shoot������rev����������
    CAN_cmd_gimbal(yaw_val, pitch_val, shoot_val, 0);

    // Ħ����
    if (wheel_enable) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    }
}

void gimbal_task_entry(void const *argument) {
    init_pid();          // PID�ṹ���ʼ��
    init_motor_status(); // ��ʼ�����ڻ�ȡ���״̬�Ľṹ�壬��ȡ�����ʼλ��
    while (1) {
        handle_rc();     // ����ң��������
        calc_pid();      // ����PID
        motor_control(); // ���Ƶ��
        osDelay(1);
    }
}
