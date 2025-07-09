#include "gimbal_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "struct_typedef.h"
#include "tim.h"
#include <math.h>

#include "CAN_receive.h"
#include "dbus_task.h"

// pitch电机参数
#define PTICH_MAX_SPEED 100.0f // pitch满杆量速度【单位：角度/s】
#define PITCH_MAX_ANGLE 50.0f  // pitch电机最大角度
// 用于计算PID的参数
#define PITCH_MAX_CURRENT 3.0f    // 最大电流【单位：A（6020电机堵转电流0.9A，CAN通信最大支持3A）】
#define PITCH_PMAX_ERR 5.0f       // 达到此误差后，P输出拉满【单位：角度】
#define PITCH_STEADY_CURRENT 0.6f // 稳态电流【单位：A】
#define PITCH_STEADY_ERR 20.0f    // 稳态误差（PID不加I时的误差）【单位：角度】
#define PITCH_STEADY_T 0.1f       // 达到稳态的时间常数【单位：s】
#define PITCH_D_RATIO 0.0f

// yaw电机参数
#define YAW_MAX_SPEED 360.0f // yaw满杆量速度【单位：角度/s】
// 用于计算PID的参数
#define YAW_MAX_CURRENT 3.0f // 最大电流【单位：A（6020电机堵转电流0.9A，CAN通信最大支持3A）】
#define YAW_PMAX_ERR 10.0f   // 达到此误差后，P输出拉满【单位：角度
#define YAW_D_RATIO (-0.5f)

// 拨弹电机
#define SHOOT_FREQ 1 // 发射弹丸频率（每秒钟多少发）
// 用于计算PID的参数
#define SHOOT_MAX_CURRENT 5.0f                     // 最大电流【单位：A（6002电机最大10A）】
#define SHOOT_SPEED (360.0f / 8 * SHOOT_FREQ * 36) // 拨弹电机速度【单位：角度/s】（8为转一圈发射的弹丸数，36为6002电机减速比）
#define SHOOT_PMAX_SPEED SHOOT_SPEED               // 达到此速度后，P输出拉满【单位：角度/s】

// 常量
#define DT 0.001f // 闭环控制周期【单位：ms】

// 目标值
fp32 yaw_angle_set;   // 单位：角度
fp32 pitch_angle_set; // 单位：角度
fp32 shoot_speed_set; // 单位：角度/s
uint8_t wheel_enable; // 摩擦轮使能

// 电机初始位置
fp32 yaw_offset;
fp32 pitch_offset;

// 用于获取电机状态
const motor_measure_t *yaw_motor_status;
const motor_measure_t *pitch_motor_status;
const motor_measure_t *shoot_motor_status;

// PID结构体
pid_type_def yaw_angle_pid;
pid_type_def pitch_angle_pid;
pid_type_def shoot_speed_pid;

static void init_pid() {
    // 函数参数含义: {P, I, D}, MAX_OUT, MAX_I

    // yaw电机 - 位置PID（PID输入角度，输出电流）
    const fp32 YAW_KP = YAW_MAX_CURRENT / YAW_PMAX_ERR;
    const fp32 YAW_KI = 0;
    const fp32 YAW_KD = YAW_KP * YAW_D_RATIO;
    const fp32 yaw_pid_para[3] = {YAW_KP, YAW_KI, YAW_KD};
    PID_init(&yaw_angle_pid, PID_POSITION, yaw_pid_para, YAW_MAX_CURRENT, 0);

    // pitch电机 - 位置PID（PID输入角度，输出电流）
    const fp32 PITCH_KP = PITCH_MAX_CURRENT / PITCH_PMAX_ERR;
    const fp32 PITCH_KI = PITCH_STEADY_CURRENT / PITCH_STEADY_ERR * DT / PITCH_STEADY_T;
    const fp32 PITCH_KD = PITCH_KP * PITCH_D_RATIO;
    const fp32 pitch_pid_para[3] = {PITCH_KP, PITCH_KI, PITCH_KD};
    PID_init(&pitch_angle_pid, PID_POSITION, pitch_pid_para, PITCH_MAX_CURRENT, PITCH_STEADY_CURRENT * 1.2f);

    // 拨弹电机 - 速度PID（PID输入速度，输出电流）
    const fp32 SHOOT_KP = SHOOT_MAX_CURRENT / SHOOT_PMAX_SPEED;
    const fp32 SHOOT_KI = 0;
    const fp32 SHOOT_KD = 0;
    const fp32 shoot_pid_para[3] = {SHOOT_KP, SHOOT_KI, SHOOT_KD};
    PID_init(&shoot_speed_pid, PID_POSITION, shoot_pid_para, SHOOT_MAX_CURRENT, 0);
}

static void init_motor_status() {
    // 用于获取电机运行状态
    yaw_motor_status = get_yaw_gimbal_motor_measure_point();
    pitch_motor_status = get_pitch_gimbal_motor_measure_point();
    shoot_motor_status = get_trigger_motor_measure_point(); // 拨弹电机

    // 确保收到电机数据
    osDelay(500);

    // 获取电机初始位置
    yaw_offset = yaw_motor_status->ecd / 8191.0f * 360.0f;
    pitch_offset = pitch_motor_status->ecd / 8191.0f * 360.0f;
}

static void handle_rc() {
    if (rc_data.is_connected == 0) { // 遥控器未连接
        wheel_enable = 0;
        shoot_speed_set = 0;
        return;
    }

    // pitch电机
    pitch_angle_set += rc_data.pitch * PTICH_MAX_SPEED * DT;
    if (pitch_angle_set > PITCH_MAX_ANGLE)
        pitch_angle_set = PITCH_MAX_ANGLE;
    if (pitch_angle_set < 0)
        pitch_angle_set = 0;

    // yaw电机
    yaw_angle_set += (-rc_data.yaw) * YAW_MAX_SPEED * DT;
    if (yaw_angle_set > 360)
        yaw_angle_set -= 360;
    if (yaw_angle_set < 0)
        yaw_angle_set += 360;

    // 摩擦轮、拨弹电机，受左拨杆控制
    if (rc_data.switch_left == UP) { // 关闭：摩擦轮关、拨弹电机关
        wheel_enable = 0;
        shoot_speed_set = 0;
    } else if (rc_data.switch_left == MID) { // 准备：摩擦轮开，拨弹电机关
        wheel_enable = 1;
        shoot_speed_set = 0;
    } else if (rc_data.switch_left == DOWN) { // 发射：摩擦轮开，拨弹电机开
        wheel_enable = 1;
        shoot_speed_set = SHOOT_SPEED;
    }
}

static void calc_pid() {
    // yaw pid 位置环
    // ecd为电机编码器值，0~8191 对应 0~360度
    fp32 yaw_angle_now = yaw_motor_status->ecd / 8191.0f * 360.0f;       // yaw当前角度
    fp32 yaw_angle_ex = yaw_angle_set + yaw_offset;                      // yaw期望角度
    fp32 yaw_err = fmodf(yaw_angle_ex - yaw_angle_now + 180, 360) - 180; // 最短路径误差
    PID_calc(&yaw_angle_pid, 0, yaw_err);

    // pitch pid 位置环
    // ecd为电机编码器值，0~8191 对应 0~360度
    fp32 pitch_angle_now = pitch_motor_status->ecd / 8191.0f * 360.0f;         // pitch当前角度
    fp32 pitch_angle_ex = pitch_angle_set + pitch_offset;                      // pitch期望角度
    fp32 pitch_err = fmodf(pitch_angle_ex - pitch_angle_now + 180, 360) - 180; // 最短路径误差
    PID_calc(&pitch_angle_pid, 0, pitch_err);

    // shoot pid 速度环
    // ecd为电机编码器值，0~8191 对应 0~360度
    fp32 shoot_speed_now = shoot_motor_status->speed_rpm / 60.0f * 360.0f; // shoot速度当前值【单位：角度/s】
    PID_calc(&shoot_speed_pid, shoot_speed_now, shoot_speed_set);
}

static void motor_control() {
    // 6020电机：-3A~0~3A => -16384~0~16384
    // 2006电机（减速比36）：-10~0~10A => -10000~0~10000
    int16_t yaw_val = yaw_angle_pid.out / 3.0f * 16384.0f;
    int16_t pitch_val = pitch_angle_pid.out / 3.0f * 16384.0f;
    int16_t shoot_val = shoot_speed_pid.out / 10.0f * 10000.0f;
    // CAN发送yaw pitch shoot电流，rev参数无作用
    CAN_cmd_gimbal(yaw_val, pitch_val, shoot_val, 0);

    // 摩擦轮
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
    init_pid();          // PID结构体初始化
    init_motor_status(); // 初始化用于获取电机状态的结构体，获取电机初始位置
    while (1) {
        handle_rc();     // 处理遥控器数据
        calc_pid();      // 计算PID
        motor_control(); // 控制电机
        osDelay(1);
    }
}
