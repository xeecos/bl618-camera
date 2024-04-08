#include "usbd_desc.h"
#include "usbd_cdc.h"

#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x02
#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

volatile bool ep_tx_busy_flag = false;
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[2048];
void usbd_event_handler(uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            /* setup first out ep read transfer */
            usbd_ep_start_read(CDC_OUT_EP, read_buffer, 2048);
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

void usbd_cdc_acm_bulk_out(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual out len:%d\r\n", nbytes);
    // for (int i = 0; i < 100; i++) {
    //     printf("%02x ", read_buffer[i]);
    // }
    // printf("\r\n");
    /* setup next out ep read transfer */
    usbd_ep_start_read(CDC_OUT_EP, read_buffer, 2048);
}

void usbd_cdc_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual in len:%d\r\n", nbytes);

    if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(CDC_IN_EP, NULL, 0);
    } else {
        ep_tx_busy_flag = false;
    }
}

struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

struct usbd_interface intf0;
struct usbd_interface intf1;

void video_init()
{
    usbd_desc_register(video_descriptor);
    struct usb_msosv1_descriptor *desc = {"MSFT100",0xA0};
    usbd_msosv1_desc_register(desc);
    // usbd_add_interface(usbd_video_init_intf(&intf0, INTERVAL, MAX_FRAME_SIZE, MAX_PAYLOAD_SIZE));
    // usbd_add_interface(usbd_video_init_intf(&intf1, INTERVAL, MAX_FRAME_SIZE, MAX_PAYLOAD_SIZE));
    // usbd_add_endpoint(&video_in_ep);

    usbd_add_interface(usbd_cdc_acm_init_intf(&intf0));
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf1));
    usbd_add_endpoint(&cdc_out_ep);
    usbd_add_endpoint(&cdc_in_ep);

    usbd_initialize();
}

// USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t packet_buffer[10 * 1024];

// void video_test()
// {
//     uint32_t out_len;
//     memset(packet_buffer, 0, 10 * 1024);
//     while (1) {
//         if (tx_flag) {
//             // usbd_video_mjpeg_payload_fill((uint8_t *)jpeg_data, sizeof(jpeg_data), packet_buffer, &out_len);
//             iso_tx_busy = true;
//             usbd_ep_start_write(VIDEO_IN_EP, packet_buffer, out_len);
//             while (iso_tx_busy) {
//                 if (tx_flag == 0) {
//                     break;
//                 }
//             }
//         }
//     }
// }