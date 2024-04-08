#include "bflb_mtimer.h"
#include "board.h"
#include "cam.h"
// #include "usbd_desc.h"

int main(void)
{
    board_init();
    // int len = 0;
    // while(1)
    // {
    //     if(video_descriptor[len]==0xa&&video_descriptor[len+1]==0xa&&video_descriptor[len+2]==0xa&&video_descriptor[len+3]==0xa)
    //     {
    //         break;
    //     }
    //     len++;
    // }
    cam_init();
    while (1) 
    {
        // printf("size:%d\n",len);
        // bflb_gpio_set(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(1000);
        // bflb_gpio_reset(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(100);
    }
}