
#include "board.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"

extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);
int main(void)
{
    board_init();
    // cam_init();
    // wifi_init();
    cdc_acm_init();
    // vTaskStartScheduler();
    while(1)
    {
        cdc_acm_data_send_with_dtr_test();
        bflb_mtimer_delay_ms(500);
    }
}
