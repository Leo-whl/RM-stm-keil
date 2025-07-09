#include "bmi088.h"
#include "main.h"
#include "spi.h"
#include "MahonyAHRS.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tim.h"
#define BMI088_ACCEL_3G_SEN 0.0008974358974f    //���������Ҳ��֪��������
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f    //���������Ҳ��֪��������
float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
float accelerometer[3];
float gyro[3];
float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};
bmi_value zitai;
uint8_t i = 0;
uint8_t buf_bmi[8]={0,0,0,0,0,0,0,0};
uint8_t pTxData;
uint8_t pRxData;
float yaw_du;


void bmi_task_entry(void *argument)
{
	  //��9�д��룬���ַ0x7E��д��0xB6ֵ�����ٶȼ������λ��ʹ���ٶȼƸ����Ĵ����ָ�ΪĬ��ֵ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4��0��Ƭѡ���ٶȼ�
    pTxData = (0x7E & 0x7F);    //Bit #0��Bit #1-7��Bit #0��0����ʾд
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    osDelay(1);    //��ʱ1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4��1��ȡ��Ƭѡ���ٶȼ�
 
    //���ٶȼƸ�λ��Ĭ������ͣģʽ����9�д��룬���ַ0x7D��д��0x04ֵ��ʹ���ٶȼƽ�������ģʽ
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4��0��Ƭѡ���ٶȼ�
    pTxData = (0x7D & 0x7F);    //Bit #0��Bit #1-7��Bit #0��0����ʾд
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    pTxData = 0x04;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    osDelay(1);    //��ʱ1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4��1��ȡ��Ƭѡ���ٶȼ�
 
    //��9�д��룬���ַ0x14��д��0xB6ֵ�������������λ��ʹ�����Ǹ����Ĵ����ָ�ΪĬ��ֵ
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0��0��Ƭѡ������
    pTxData = (0x14 & 0x7F);    //Bit #0��Bit #1-7��Bit #0��0����ʾд
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    osDelay(30);    //��ʱ30ms
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0��1��ȡ��Ƭѡ������
	HAL_TIM_Base_Start_IT(&htim5);

 while (1)
  {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4��0��Ƭѡ���ٶȼ�
    pTxData = (0x12 | 0x80);    //Bit #0��Bit #1-7��Bit #0��1����ʾ��
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15����Чֵ
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //�ȴ�SPI�������
    i = 0;
    while (i < 6)
    {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #16-23���Ĵ���0x12��ֵ��Ȼ���ǼĴ���0x13��0x14��0x15��0x16��0x17��ֵ
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //�ȴ�SPI�������
    	buf_bmi[i] = pRxData;
        i++;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4��1��ȡ��Ƭѡ���ٶȼ�
    accelerometer[0] = ((int16_t)((buf_bmi[1]) << 8) | buf_bmi[0]) * BMI088_ACCEL_SEN;
    accelerometer[1] = ((int16_t)((buf_bmi[3]) << 8) | buf_bmi[2]) * BMI088_ACCEL_SEN;
    accelerometer[2] = ((int16_t)((buf_bmi[5]) << 8) | buf_bmi[4]) * BMI088_ACCEL_SEN;
 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0��0��Ƭѡ������
    pTxData = (0x00 | 0x80);    //Bit #0��Bit #1-7��Bit #0��1����ʾ��
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
    i = 0;
    while (i < 8)
    {
    	HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15���Ĵ���0x00��ֵ��Ȼ���ǼĴ���0x01��0x02��0x03��0x04��0x05��0x06��0x07��ֵ
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //�ȴ�SPI�������
    	buf_bmi[i] = pRxData;
    	i++;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0��1��ȡ��Ƭѡ������
    if(buf_bmi[0] == 0x0F)	//buf[0]����GYRO_CHIP_ID��Ӧ��Ϊ0x0F���ж����Ƕ�ȡ�����ǲ��������ǵ�ֵ��
    {
    	gyro[0] = ((int16_t)((buf_bmi[3]) << 8) | buf_bmi[2]) * BMI088_GYRO_SEN;
    	gyro[1] = ((int16_t)((buf_bmi[5]) << 8) | buf_bmi[4]) * BMI088_GYRO_SEN;
    	gyro[2] = ((int16_t)((buf_bmi[7]) << 8) | buf_bmi[6]) * BMI088_GYRO_SEN;
    }
 
	MahonyAHRSupdateIMU(gyro[0], gyro[1], gyro[2], accelerometer[0], accelerometer[1], accelerometer[2],quat);    //�Եõ��ļ��ٶȼơ����������ݽ��м���õ���Ԫ��
	get_angle(quat, &zitai.yaw, &zitai.roll,&zitai.pitch);  	//����Ԫ������õ�ŷ����
		
	
	 yaw_du = zitai.yaw*180/3.141592653f;
  }


}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4��0��Ƭѡ���ٶȼ�
//    pTxData = (0x12 | 0x80);    //Bit #0��Bit #1-7��Bit #0��1����ʾ��
//    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
//    HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15����Чֵ
//    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //�ȴ�SPI�������
//    i = 0;
//    while (i < 6)
//    {
//        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #16-23���Ĵ���0x12��ֵ��Ȼ���ǼĴ���0x13��0x14��0x15��0x16��0x17��ֵ
//    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //�ȴ�SPI�������
//    	buf[i] = pRxData;
//        i++;
//    }
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4��1��ȡ��Ƭѡ���ٶȼ�
//    accelerometer[0] = ((int16_t)((buf[1]) << 8) | buf[0]) * BMI088_ACCEL_SEN;
//    accelerometer[1] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_ACCEL_SEN;
//    accelerometer[2] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_ACCEL_SEN;
// 
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0��0��Ƭѡ������
//    pTxData = (0x00 | 0x80);    //Bit #0��Bit #1-7��Bit #0��1����ʾ��
//    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //�ȴ�SPI�������
//    i = 0;
//    while (i < 8)
//    {
//    	HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15���Ĵ���0x00��ֵ��Ȼ���ǼĴ���0x01��0x02��0x03��0x04��0x05��0x06��0x07��ֵ
//    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //�ȴ�SPI�������
//    	buf[i] = pRxData;
//    	i++;
//    }
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0��1��ȡ��Ƭѡ������
//    if(buf[0] == 0x0F)	//buf[0]����GYRO_CHIP_ID��Ӧ��Ϊ0x0F���ж����Ƕ�ȡ�����ǲ��������ǵ�ֵ��
//    {
//    	gyro[0] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_GYRO_SEN;
//    	gyro[1] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_GYRO_SEN;
//    	gyro[2] = ((int16_t)((buf[7]) << 8) | buf[6]) * BMI088_GYRO_SEN;
//    }
// 
//	MahonyAHRSupdateIMU(gyro[0], gyro[1], gyro[2], accelerometer[0], accelerometer[1], accelerometer[2],quat);    //�Եõ��ļ��ٶȼơ����������ݽ��м���õ���Ԫ��
//	get_angle(quat, &zitai.pitch, &zitai.roll,&zitai.yaw);    //����Ԫ������õ�ŷ����
//	 


//	
//  }

