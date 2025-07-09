//
// Created by 27459 on 25-5-27.
//

#ifndef CAN_R_H
#define CAN_R_H

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"

typedef struct
{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//机械角度
    int16_t  rotor_speed;//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temp;//温度
}moto_info_t;


extern moto_info_t motor_yaw_info;
extern moto_info_t trigger_info;
extern moto_info_t shoot_info_1;
extern moto_info_t shoot_info_2;
void can_filter_init(void);
void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1);
void set_trigger_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1);
void set_shoot_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1,int16_t v2,int16_t v3);

#endif //CAN_R_H
