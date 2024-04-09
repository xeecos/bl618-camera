#include "bflb_mtimer.h"
#include "board.h"
#include "cam.h"

extern void video_init(void);
int main(void)
{
    board_init();
    cam_init();
    video_init();
}