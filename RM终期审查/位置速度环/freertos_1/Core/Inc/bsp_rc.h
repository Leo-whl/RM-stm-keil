#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"
#include "usart.h"


#define SBUS_RX_BUF_NUM     (50)
#define RC_FRAME_LENGTH      (25)
#define DBUS_HUART       huart3

typedef struct
{
		uint16_t CH1;
		uint16_t CH2;
		uint16_t CH3;
		uint16_t CH4;
		uint16_t CH5;
		uint16_t CH6;
    uint16_t CH7;
    uint16_t CH8;
    uint16_t CH9;
    uint16_t CH10;
    uint16_t CH11;
    uint16_t CH12;
    uint16_t CH13;
    uint16_t CH14;
    uint16_t CH15;
    uint16_t CH16;
		uint8_t ConnectState;
}sbus_ch_t;

void remote_control_init(void);


extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
#endif
