#include "bsp.h"

void BSP_Init() {
    // BSP_CAN_Init();
    BSP_DWT_Init();
    BSP_LED_Init();
    BSP_UART_Init();
}
