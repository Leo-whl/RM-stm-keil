#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
#include "math.h"
#define PI					3.14159265358979f

//yaw encode angle close-loop PID params, max out and max iout
//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
void abs_limit(fp32 *num, fp32 Limit);
#endif
