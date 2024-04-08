/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usbd_cdc.h"

const char *stop_name[] = { "1", "1.5", "2" };
const char *parity_name[] = { "N", "O", "E", "M", "S" };

static int cdc_acm_class_interface_request_handler(struct usb_setup_packet *setup, uint8_t **data, uint32_t *len)
{
    USB_LOG_DBG("CDC Class request: "
                "bRequest 0x%02x\r\n",
                setup->bRequest);


    return 0;
}

struct usbd_interface *usbd_cdc_acm_init_intf(struct usbd_interface *intf)
{
    intf->class_interface_handler = cdc_acm_class_interface_request_handler;
    intf->class_endpoint_handler = NULL;
    intf->vendor_handler = NULL;
    intf->notify_handler = NULL;

    return intf;
}

