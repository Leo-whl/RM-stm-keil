
#include "bsp_can.h"

#include "can.h"
#include "gpio.h"
#include "main.h"


void can_Init(CAN_HandleTypeDef*hcan)
{
	//配置can的过滤器
		  CAN_FilterTypeDef  can_filter;
		  can_filter.FilterBank = 0;                       // filter 0
		  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
		  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
		  can_filter.FilterIdHigh = 0;
		  can_filter.FilterIdLow  = 0;
		  can_filter.FilterMaskIdHigh = 0;
		  can_filter.FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
		  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
		  can_filter.FilterActivation = ENABLE;           // enable can filter
		  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode

		  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter


	HAL_CAN_Start(&hcan1);//开启can
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//开启在fifo0中的接收中断


}

void boxSend(uint16_t I1,uint16_t I2,uint16_t I3,uint16_t I4)//电压控制1，范围在-25000到25000
{
	uint8_t Data[8]={0};
	CAN_TxHeaderTypeDef txheader;//创建发送报文结构体
	txheader.DLC=8;
	txheader.IDE=CAN_ID_STD;
	txheader.RTR=CAN_RTR_DATA;
//	txheader.StdId=(ID==0)?(0x1ff):(0x2ff);//判断ID是哪个
	txheader.StdId=0x1ff;
	Data[0]=(uint8_t)(I1>>8);
	Data[1]=(uint8_t)I1;

	Data[2]=(uint8_t)(I2>>8);
	Data[3]=(uint8_t)I2;

	Data[4]=(uint8_t)(I3>>8);
	Data[5]=(uint8_t)I3;

	Data[6]=(uint8_t)(I4>>8);
	Data[7]=(uint8_t)I4;

	HAL_CAN_AddTxMessage(&hcan1,&txheader,Data,(uint32_t*)CAN_TX_MAILBOX0);
}


void motorset(Motor_t*Receive,uint8_t Data[])
{
	Receive->angle=(Data[0]<<8)|Data[1];//转子机械角度
	Receive->speed=(Data[2]<<8)|Data[3];//转速
	Receive->torque=(Data[4]<<8)|Data[5];//输出转矩
	Receive->temp=Data[6];//温度
}

Motor_t GM6020[4];//控制四个电机

//void boxReceive(void)
//{
//	uint8_t ReceiveData[8]={0};
//	CAN_RxHeaderTypeDef Rxheader;//创建接收报文结构体
//	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rxheader,ReceiveData);//将信息接收到fifo0邮箱
//	switch(Rxheader.StdId)//判断ID
//	{
//	case 0x201:
//	{
//		motorset(&GM6020[0],ReceiveData);//处理接收到的数据，拿出来
//		break;
//	}
//
//	}
//
//}


//在fifo0的中断回调里面完成数据接收

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	if(hcan->Instance==CAN1)
	{
	    uint8_t ReceiveData[8]={0};
		CAN_RxHeaderTypeDef Rxheader;//创建接收报文结构体
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rxheader,ReceiveData);//将信息接收到fifo0邮箱
		switch(Rxheader.StdId)//判断ID
		{
		case 0x205:
		{
			motorset(&GM6020[0],ReceiveData);//处理接收到的数据，拿出来
			break;
		}

		}
	}
}






/*
 * bsp_can.c
 *
 *  Created on: Jun 3, 2024
 *      Author: yu
 */

