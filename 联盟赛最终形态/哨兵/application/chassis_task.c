#include "chassis_task.h"
#include "can.h"
#include "cmsis_os.h"
#include "main.h"
#include <math.h>

#include "dbus_task.h"
#include "pid.h"
#include "struct_typedef.h"

// 底盘结构参数
#define WHEEL_RADIUS 0.053f   // 轮子半径【单位：m】
#define CHASSIS_RADIUS 0.428f // 底盘半径【单位：m】

// 底盘速度参数
#define VX_MAX 1.0f  // 前后运动最大速度【单位：m/s】
#define VY_MAX 1.0f  // 左右平移最大单位【单位：m/s】
#define VR_MAX 30.0f // 旋转最大速度【单位：rpm】

// 6020电机参数
#define M6020_MAX_VOLTAGE 12.0f          // 最大电压【单位；V（假设满电压24V）】
#define M6020_PMAX_ERR 30.0f             // 达到此误差后，P输出拉满【单位：角度】
#define M6020_STEADY_VOLTAGE_RATIO 0.25f // 稳态电压比例
#define M6020_STEADY_ERR 10.0f           // 稳态误差（PID不加I时的误差）【单位：角度】
#define M6020_STEADY_T 0.1f              // 达到稳态的时间常数【单位：s】
#define M6020A_OFFSET 90.0f
#define M6020B_OFFSET 150.0f

// 3508电机参数
#define M3508_MAX_CURRENT 2.5f // 最大电流【单位；A（3508电机堵转电流2.5A，CAN通信最大支持20A）】
#define M3508_PMAX_ERR 30.0f   // 达到此误差后，P输出拉满【单位：输出转速rpm】
#define M3508_D_RATIO 0.0f
#define M3508_RATIO (3591.0f / 187.0f) // 3508电机减速比

// 常量
#define DT 0.001f // 闭环控制周期【单位：ms】
#define PI 3.1415926f

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }
#define ecd_to_angle(ecd) (ecd / 8191.0f * 360.0f) // 编码器值转角度

// 电机状态
typedef struct {
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
} motor_status_t;

// 底盘速度分量
typedef struct {
    fp32 x; // 【单位：rpm，等效轮子速度】
    fp32 y; // 【单位：rpm，等效轮子速度】
    fp32 r; // 【单位：rpm，等效轮子速度】
} v_t;

typedef struct m6020 {
    motor_status_t status;  // 从CAN获取电机状态
    pid_type_def angle_pid; // PID结构体
    fp32 angle_set;         // 角度目标值【单位：角度】
} m6020_t;

typedef struct m3508 {
    motor_status_t status; // 从CAN获取电机状态
    pid_type_def rpm_pid;  // PID结构体
    fp32 rpm_set;          // 速度目标值【单位：rpm，减速前的速度】
    int8_t dir_set;        // 电机旋转方向，1为正转，-1为反转
} m3508_t;

v_t v;
m6020_t m6020a, m6020b;
m3508_t m3508a, m3508b;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);

    switch (header.StdId) {
    case 0x205: // 6020电机，ID为1
        get_motor_measure(&m6020a.status, data);
        break;
    case 0x206: // 6020电机，ID为2
        get_motor_measure(&m6020b.status, data);
        break;
    case 0x207: // 3508电机，ID为7
        get_motor_measure(&m3508a.status, data);
        break;
    case 0x208: // 3508电机，ID为8
        get_motor_measure(&m3508b.status, data);
        break;
    }
}

// 6020为舵电机，3508为轮电机
static void can_send_current(fp32 m6020a, fp32 m6020b, fp32 m3508a, fp32 m3508b) {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t send_mail_box;

    // 6020电机：电压控制，-25000~0~25000（假设满电压为24V）
    int16_t m6020a_val = m6020a / 24.0f * 25000.0f;
    int16_t m6020b_val = m6020b / 24.0f * 25000.0f;

    // 3508电机（C620电调）：-20~0~20A => -16384~0~16384
    int16_t m3508a_val = m3508a / 20.0f * 16384.0f;
    int16_t m3508b_val = m3508b / 20.0f * 16384.0f;

    header.StdId = 0X1FF;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 0x08;
    data[0] = m6020a_val >> 8; // 6020电机，ID：1
    data[1] = m6020a_val;
    data[2] = m6020b_val >> 8; // 6020电机，ID：2
    data[3] = m6020b_val;
    data[4] = m3508a_val >> 8; // 3508电机，ID：7
    data[5] = m3508a_val;
    data[6] = m3508b_val >> 8; // 3508电机，ID：8
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

    float err = fmodf(set - ref + 540.0f, 360.0f) - 180.0f; // 最短路径误差

    return err;
}

void chassis_task_entry(void *argument) {
    // 初始化PID参数
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
        // 检查遥控器连接
        if (rc_data.is_connected == 0) {
            can_send_current(0, 0, 0, 0);
            osDelay(1);
            continue;
        }

        // 根据运动模式，计算底盘速度分量
        if (rc_data.switch_right == UP) { // 前后移动、左右旋转
            v.x = 0;
            v.y = rc_data.y * VX_MAX / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            v.r = rc_data.x * VR_MAX / (CHASSIS_RADIUS / WHEEL_RADIUS) * 60.0f;
        } else if (rc_data.switch_right == MID || rc_data.switch_right == DOWN) { // 前后左右平移
            v.x = rc_data.x * VX_MAX / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            v.y = rc_data.y * VY_MAX / (WHEEL_RADIUS * 2 * PI) * 60.0f;
            v.r = 0;
        }

        // 根据速度分量，计算电机目标值
        m6020a.angle_set = atan2(v.x, v.y - v.r) / PI * 180.0f + M6020A_OFFSET;
        m6020b.angle_set = atan2(v.x, v.y + v.r) / PI * 180.0f + M6020B_OFFSET;
        m3508a.rpm_set = sqrtf(powf(v.y - v.r, 2) + powf(v.x, 2)) * M3508_RATIO;
        m3508b.rpm_set = -sqrtf(powf(v.y + v.r, 2) + powf(v.x, 2)) * M3508_RATIO;

        // 计算PID
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

        // CAN发送电机电流
        can_send_current(m6020a.angle_pid.out, m6020b.angle_pid.out, m3508a.rpm_pid.out, m3508b.rpm_pid.out);

        osDelay(1);
    }
}
