#include "FreeRTOS.h"
#include "wifi.h"
#include "task.h"
#include "bflb_mtimer.h"
#include "bflb_gpio.h"
#include "board.h"
#include "cam.h"
#include "microphone.h"
#include "usb_uart.h"
#include "config.h"
int main(void)
{
    board_init();
    cdc_acm_init();
    bflb_mtimer_delay_ms(200);
    cam_init();
    microphone_init();
    wifi_init();
    vTaskStartScheduler();
    while (1) {
        bflb_mtimer_delay_ms(100);
    }
}