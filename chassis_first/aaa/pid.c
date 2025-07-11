//
// Created by 27459 on 25-5-27.
//


#include "pid.h"

pid_struct_t gimbal_yaw_speed_pid;
pid_struct_t gimbal_yaw_angle_pid;
pid_struct_t trigger_speed_pid;
pid_struct_t shoot_speed_pid_1;
pid_struct_t shoot_speed_pid_2;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID初始化函数
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;
}
void LIMIT_MIN_MAX(float a,float b,float c)
{
    if(a<b) a =b;
    if(a>c) a =c;
}

float pid_calc(pid_struct_t *pid, float ref, float fdb)//PID运算函数
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    pid->p_out  = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
    LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
    return pid->output;
}

void gimbal_PID_init()//角度环和速度环的PID初始化,只是初测出来的数据，具体还需要测试
{
    pid_init(&gimbal_yaw_speed_pid, 50, 0.1, 0.2, 3000, 30000);//P=30,I=0,D=0
    pid_init(&gimbal_yaw_angle_pid, 400, 0, 0, 0, 320);//P=400,I=0,D=0
}
void trigger_PID_init()
{
    pid_init(&trigger_speed_pid, 5, 0.0, 0.0, 3000, 8084);
}
void shoot_PID_init()
{
    pid_init(&shoot_speed_pid_1, 5, 0.0, 0.0, 3000, 16384);
    pid_init(&shoot_speed_pid_2, 5, 0.0, 0.0, 3000, 16384);
}