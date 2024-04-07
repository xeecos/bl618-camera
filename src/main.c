#include "bflb_mtimer.h"
#include "cam.h"
#include "board.h"
int main(void)
{
    board_init();
    cam_init();

    while (1) 
    {
        bflb_mtimer_delay_ms(1000);
    }
}