#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
// #include "cam.h"
#include "wifi.h"

int main(void)
{
    board_init();
    // cam_init();
    wifi_init();
    vTaskStartScheduler();
    while(1)
    {
    }
}
