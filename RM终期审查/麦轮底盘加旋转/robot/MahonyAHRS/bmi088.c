#include "bmi088.h"
#include "main.h"
#include "spi.h"
#include "MahonyAHRS.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tim.h"
#define BMI088_ACCEL_3G_SEN 0.0008974358974f    //这个数字我也不知道哪来的
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f    //这个数字我也不知道哪来的
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
	  //这9行代码，向地址0x7E处写入0xB6值，加速度计软件复位，使加速度计各个寄存器恢复为默认值
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x7E & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    osDelay(1);    //延时1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
 
    //加速度计复位后默认是暂停模式，这9行代码，向地址0x7D处写入0x04值，使加速度计进入正常模式
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x7D & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0x04;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    osDelay(1);    //延时1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
 
    //这9行代码，向地址0x14处写入0xB6值，陀螺仪软件复位，使陀螺仪各个寄存器恢复为默认值
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0置0，片选陀螺仪
    pTxData = (0x14 & 0x7F);    //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    pTxData = 0xB6;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    osDelay(30);    //延时30ms
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0置1，取消片选陀螺仪
	HAL_TIM_Base_Start_IT(&htim5);

 while (1)
  {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
    pTxData = (0x12 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，无效值
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    i = 0;
    while (i < 6)
    {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #16-23，寄存器0x12的值，然后是寄存器0x13、0x14、0x15、0x16、0x17的值
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    	buf_bmi[i] = pRxData;
        i++;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
    accelerometer[0] = ((int16_t)((buf_bmi[1]) << 8) | buf_bmi[0]) * BMI088_ACCEL_SEN;
    accelerometer[1] = ((int16_t)((buf_bmi[3]) << 8) | buf_bmi[2]) * BMI088_ACCEL_SEN;
    accelerometer[2] = ((int16_t)((buf_bmi[5]) << 8) | buf_bmi[4]) * BMI088_ACCEL_SEN;
 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0置0，片选陀螺仪
    pTxData = (0x00 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
    i = 0;
    while (i < 8)
    {
    	HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，寄存器0x00的值，然后是寄存器0x01、0x02、0x03、0x04、0x05、0x06、0x07的值
    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
    	buf_bmi[i] = pRxData;
    	i++;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0置1，取消片选陀螺仪
    if(buf_bmi[0] == 0x0F)	//buf[0]储存GYRO_CHIP_ID，应该为0x0F，判断我们读取到的是不是陀螺仪的值。
    {
    	gyro[0] = ((int16_t)((buf_bmi[3]) << 8) | buf_bmi[2]) * BMI088_GYRO_SEN;
    	gyro[1] = ((int16_t)((buf_bmi[5]) << 8) | buf_bmi[4]) * BMI088_GYRO_SEN;
    	gyro[2] = ((int16_t)((buf_bmi[7]) << 8) | buf_bmi[6]) * BMI088_GYRO_SEN;
    }
 
	MahonyAHRSupdateIMU(gyro[0], gyro[1], gyro[2], accelerometer[0], accelerometer[1], accelerometer[2],quat);    //对得到的加速度计、陀螺仪数据进行计算得到四元数
	get_angle(quat, &zitai.yaw, &zitai.roll,&zitai.pitch);  	//对四元数计算得到欧拉角
		
	
	 yaw_du = zitai.yaw*180/3.141592653f;
  }


}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    //PA4置0，片选加速度计
//    pTxData = (0x12 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
//    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
//    HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，无效值
//    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
//    i = 0;
//    while (i < 6)
//    {
//        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #16-23，寄存器0x12的值，然后是寄存器0x13、0x14、0x15、0x16、0x17的值
//    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
//    	buf[i] = pRxData;
//        i++;
//    }
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    //PA4置1，取消片选加速度计
//    accelerometer[0] = ((int16_t)((buf[1]) << 8) | buf[0]) * BMI088_ACCEL_SEN;
//    accelerometer[1] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_ACCEL_SEN;
//    accelerometer[2] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_ACCEL_SEN;
// 
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);    //PB0置0，片选陀螺仪
//    pTxData = (0x00 | 0x80);    //Bit #0和Bit #1-7，Bit #0是1，表示读
//    HAL_SPI_Transmit(&hspi1, &pTxData, 1, 1000);
//    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    //等待SPI发送完成
//    i = 0;
//    while (i < 8)
//    {
//    	HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #8-15，寄存器0x00的值，然后是寄存器0x01、0x02、0x03、0x04、0x05、0x06、0x07的值
//    	while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //等待SPI接收完成
//    	buf[i] = pRxData;
//    	i++;
//    }
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //PB0置1，取消片选陀螺仪
//    if(buf[0] == 0x0F)	//buf[0]储存GYRO_CHIP_ID，应该为0x0F，判断我们读取到的是不是陀螺仪的值。
//    {
//    	gyro[0] = ((int16_t)((buf[3]) << 8) | buf[2]) * BMI088_GYRO_SEN;
//    	gyro[1] = ((int16_t)((buf[5]) << 8) | buf[4]) * BMI088_GYRO_SEN;
//    	gyro[2] = ((int16_t)((buf[7]) << 8) | buf[6]) * BMI088_GYRO_SEN;
//    }
// 
//	MahonyAHRSupdateIMU(gyro[0], gyro[1], gyro[2], accelerometer[0], accelerometer[1], accelerometer[2],quat);    //对得到的加速度计、陀螺仪数据进行计算得到四元数
//	get_angle(quat, &zitai.pitch, &zitai.roll,&zitai.yaw);    //对四元数计算得到欧拉角
//	 


//	
//  }

