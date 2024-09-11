
// #include "usbd_core.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wifi.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "cam.h"

extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);
int main(void)
{
    board_init();
    bflb_mtimer_delay_ms(2000);
    cam_init();
    wifi_init();
    // cdc_acm_init();
    vTaskStartScheduler();
    while(1)
    {
        // cdc_acm_data_send_with_dtr_test();
    }
}