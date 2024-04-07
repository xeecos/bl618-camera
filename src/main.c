#include "bflb_mtimer.h"
#include "board.h"
#include "cam.h"
int main(void)
{
    board_init();
    cam_init();

    while (1) 
    {
        // bflb_gpio_set(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(100);
        // bflb_gpio_reset(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(100);
    }
}