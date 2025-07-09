#include "app.hpp"


Status status;
DJ6 dj6;

static void app_can_callback(const uint32_t id, uint8_t data[8]) {
}

static void app_rc_callback(const uint8_t *data, const uint16_t size) {
    dj6.ParseData(data, size);
}

void app_init() {
    BSP_Init();
    // 设置中断回调函数
    BSP_UART_RC_SetCallback(app_rc_callback);

}
