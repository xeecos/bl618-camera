#include "wifi.h"
#include "FreeRTOS.h"

#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

#include "bl_fw_api.h"
#include "bflb_irq.h"
#include "bflb_uart.h"

#include "bl616_glb.h"
#include "rfparam_adapter.h"
#include "wifi_mgmr_ext.h"
#include "wifi_mgmr.h"
// #include "web/mlwip_https.h"
#include "service.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>

#define USER_AP_NAME "EzCam"
#define USER_AP_PASSWORD "12345678"
#define WIFI_STACK_SIZE  (1536)
#define TASK_PRIORITY_FW (16)
#define WIFI_HTTP_SERVER_STACK_SIZE  (1024 * 8)
#define HTTP_SERVERTASK_PRIORITY (15)
static TaskHandle_t wifi_fw_task;
static wifi_conf_t conf = {
    .country_code = "CN",
};
void * MuxSem_Handle = NULL;
static TaskHandle_t http_server_task_hd;

int wifi_start_firmware_task(void)
{
    printf("Starting wifi ...\r\n");

    /* enable wifi clock */

    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_IP_WIFI_PHY | GLB_AHB_CLOCK_IP_WIFI_MAC_PHY | GLB_AHB_CLOCK_IP_WIFI_PLATFORM);
    GLB_AHB_MCU_Software_Reset(GLB_AHB_MCU_SW_WIFI);

    /* set ble controller EM Size */

    GLB_Set_EM_Sel(GLB_WRAM160KB_EM0KB);

    if (0 != rfparam_init(0, NULL, 0)) {
        printf("PHY RF init failed!\r\n");
        return 0;
    }

    printf("PHY RF init success!\r\n");

    /* Enable wifi irq */

    extern void interrupt0_handler(void);
    bflb_irq_attach(WIFI_IRQn, (irq_callback)interrupt0_handler, NULL);
    bflb_irq_enable(WIFI_IRQn);

    xTaskCreate(wifi_main, (char *)"fw", WIFI_STACK_SIZE, NULL, TASK_PRIORITY_FW, &wifi_fw_task);

    return 0;
}

void wifi_event_handler(uint32_t code)
{
    switch (code) {
        case CODE_WIFI_ON_INIT_DONE: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_INIT_DONE\r\n", __func__);
            wifi_mgmr_init(&conf);
        } break;
        case CODE_WIFI_ON_MGMR_DONE: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_MGMR_DONE\r\n", __func__);
#ifdef CONFIG_CHERRYUSB
#ifndef SIMU_DATA_TEST
            printf("[USB] uvc/uav start\r\n");
            // usbh_video_test();
            //usbh_audio_test();
#endif
#endif
        } break;
        case CODE_WIFI_ON_SCAN_DONE: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_SCAN_DONE\r\n", __func__);
            wifi_mgmr_sta_scanlist();
        } break;
        case CODE_WIFI_ON_CONNECTED: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_CONNECTED\r\n", __func__);
            void mm_sec_keydump();
            mm_sec_keydump();
        } break;
        case CODE_WIFI_ON_GOT_IP: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_GOT_IP\r\n", __func__);
        } break;
        case CODE_WIFI_ON_DISCONNECT: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_DISCONNECT\r\n", __func__);
        } break;
        case CODE_WIFI_ON_AP_STARTED: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_AP_STARTED\r\n", __func__);
        } break;
        case CODE_WIFI_ON_AP_STOPPED: {
            printf("[APP] [EVT] %s, CODE_WIFI_ON_AP_STOPPED\r\n", __func__);
        } break;
        case CODE_WIFI_ON_AP_STA_ADD: {
            printf("[APP] [EVT] [AP] [ADD] %lld\r\n", xTaskGetTickCount());
        } break;
        case CODE_WIFI_ON_AP_STA_DEL: {
            printf("[APP] [EVT] [AP] [DEL] %lld\r\n", xTaskGetTickCount());
        } break;
        default: {
            printf("[APP] [EVT] Unknown code %u \r\n", code);
        }
    }
}

static int start_ap(void)
{
    wifi_mgmr_ap_params_t config = {0};

	config.channel = 3;
	config.key = USER_AP_PASSWORD;
	config.ssid = USER_AP_NAME;
	config.use_dhcpd = 1;

	if(wifi_mgmr_conf_max_sta(2) != 0){
		return 5;
	}
	if(wifi_mgmr_ap_start(&config) == 0){
		return 0;
	}
    return 0;
}

void http_server_task(void *param)
{
    start_ap();
    // mhttp_server_init();
    service_init();
}
void create_http_server_task(void)
{
    MuxSem_Handle = xSemaphoreCreateMutex();
	if (NULL != MuxSem_Handle)
	{
		printf("MuxSem_Handle creat success!\r\n");
	}

    xTaskCreate(http_server_task, (char *)"fw", WIFI_HTTP_SERVER_STACK_SIZE, NULL, HTTP_SERVERTASK_PRIORITY, &http_server_task_hd);
}
void wifi_init()
{
    tcpip_init(NULL, NULL);
    wifi_start_firmware_task();
    create_http_server_task();
}