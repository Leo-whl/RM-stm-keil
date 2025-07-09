
#include "bsp_can.h"

#include "can.h"
#include "gpio.h"
#include "main.h"


void can_Init(CAN_HandleTypeDef*hcan)
{
	//����can�Ĺ�����
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


	HAL_CAN_Start(&hcan1);//����can
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//������fifo0�еĽ����ж�


}

void boxSend(uint16_t I1,uint16_t I2,uint16_t I3,uint16_t I4)//��ѹ����1����Χ��-25000��25000
{
	uint8_t Data[8]={0};
	CAN_TxHeaderTypeDef txheader;//�������ͱ��Ľṹ��
	txheader.DLC=8;
	txheader.IDE=CAN_ID_STD;
	txheader.RTR=CAN_RTR_DATA;
//	txheader.StdId=(ID==0)?(0x1ff):(0x2ff);//�ж�ID���ĸ�
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
	Receive->angle=(Data[0]<<8)|Data[1];//ת�ӻ�е�Ƕ�
	Receive->speed=(Data[2]<<8)|Data[3];//ת��
	Receive->torque=(Data[4]<<8)|Data[5];//���ת��
	Receive->temp=Data[6];//�¶�
}

Motor_t GM6020[4];//�����ĸ����

//void boxReceive(void)
//{
//	uint8_t ReceiveData[8]={0};
//	CAN_RxHeaderTypeDef Rxheader;//�������ձ��Ľṹ��
//	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rxheader,ReceiveData);//����Ϣ���յ�fifo0����
//	switch(Rxheader.StdId)//�ж�ID
//	{
//	case 0x201:
//	{
//		motorset(&GM6020[0],ReceiveData);//������յ������ݣ��ó���
//		break;
//	}
//
//	}
//
//}


//��fifo0���жϻص�����������ݽ���

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	if(hcan->Instance==CAN1)
	{
	    uint8_t ReceiveData[8]={0};
		CAN_RxHeaderTypeDef Rxheader;//�������ձ��Ľṹ��
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rxheader,ReceiveData);//����Ϣ���յ�fifo0����
		switch(Rxheader.StdId)//�ж�ID
		{
		case 0x205:
		{
			motorset(&GM6020[0],ReceiveData);//������յ������ݣ��ó���
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

