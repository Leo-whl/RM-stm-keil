#include "bsp_led.h"
#include "cmsis_os.h"
#include "led_task.h"
#include "main.h"

#include "struct_typedef.h"

#define RGB_FLOW_COLOR_CHANGE_TIME 1000
#define RGB_FLOW_COLOR_LENGHT 6
// blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
// �� -> ��(��) -> �� -> ��(��) -> �� -> ��(��) -> ��
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

void led_RGB_flow_task(void const *argument) {
    while (1) {
        for (uint16_t i = 0; i < RGB_FLOW_COLOR_LENGHT; i++) {
            fp32 alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
            fp32 red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
            fp32 green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
            fp32 blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

            fp32 delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
            fp32 delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
            fp32 delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
            fp32 delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;

            for (uint16_t j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++) {
                alpha += delta_alpha;
                red += delta_red;
                green += delta_green;
                blue += delta_blue;

                uint32_t aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
                aRGB_led_show(aRGB);
                
                osDelay(1);
            }
        }
    }
}
