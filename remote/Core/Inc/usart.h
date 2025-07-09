/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (25)
#define DBUS_HUART       huart3
/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
typedef __packed struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;
 typedef struct
{
    uint16_t CH1;//??1??
    uint16_t CH2;//??2??
    uint16_t CH3;//??3??
    uint16_t CH4;//??4??
    uint16_t CH5;//??5??
    uint16_t CH6;//??6??
    uint16_t CH7;//??7??
    uint16_t CH8;//??8??
    uint16_t CH9;//??9??
    uint16_t CH10;//??10??
    uint16_t CH11;//??11??
    uint16_t CH12;//??12??
    uint16_t CH13;//??13??
    uint16_t CH14;//??14??
    uint16_t CH15;//??15??
    uint16_t CH16;//??16??
    uint8_t ConnectState;//??????????? 0=???,1=????
}SBUS_CH_Struct;

#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}
/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void dbus_uart_init(void);
void uart_receive_handler(UART_HandleTypeDef *huart);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

