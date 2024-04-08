#include "bflb_mtimer.h"
#include "board.h"
#include "cam.h"

extern void usbd_init(void);
extern void usbd_loop();
int main(void)
{
    board_init();
    cam_init();
    usbd_init();
    while (1) 
    {
        // printf("size:%d\n",len);
        // bflb_gpio_set(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(1000);
        // bflb_gpio_reset(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(100);
        usbd_loop();
    }
}