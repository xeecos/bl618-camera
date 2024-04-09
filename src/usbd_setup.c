#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "bflb_gpio.h"

volatile bool ep_tx_busy_flag = true;
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t frame[2048];
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
            ep_tx_busy_flag = false;
            usbd_ep_start_read(2, read_buffer, 2048);
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
    usbd_ep_start_read(0x02, read_buffer, 2048);
}

void usbd_cdc_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % 0x200) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(0x81, write_buffer, 0);
    } else {
        ep_tx_busy_flag = false;
    }
}

struct usbd_endpoint cdc_out_ep = {
    .ep_addr = 0x2,
    .ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
    .ep_addr = 0x81,
    .ep_cb = usbd_cdc_acm_bulk_in
};

struct usbd_interface intf;
extern uint16_t lineCount;
void video_loop()
{
    if(ep_tx_busy_flag==false)
    {
        ep_tx_busy_flag = true;
        usbd_ep_start_write(0x81, write_buffer, 8);
    }
    
}
void video_init()
{
    // gpio2 = bflb_device_get_by_name("gpio");
    
    usbd_desc_register(video_descriptor);
    struct usb_msosv1_descriptor desc;
    desc.string = (const uint8_t*)"MSFT100";
    desc.vendor_code = 0xA0;
    usbd_msosv1_desc_register(&desc);
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf));
    // usbd_add_interface(usbd_cdc_acm_init_intf(&intf1));
    usbd_add_endpoint(&cdc_out_ep);
    usbd_add_endpoint(&cdc_in_ep);
    usbd_initialize();

    while (1) 
    {
        // printf("size:%d\n",len);
        // bflb_gpio_set(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(1000);
        // bflb_gpio_reset(gpio, GPIO_PIN_29);
        // bflb_mtimer_delay_ms(100);
        video_loop();
    }
}