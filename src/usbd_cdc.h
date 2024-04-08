/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBD_CDC_H
#define USBD_CDC_H

#include "usb_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Init cdc acm interface driver */
struct usbd_interface *usbd_cdc_acm_init_intf(struct usbd_interface *intf);


#ifdef __cplusplus
}
#endif

#endif /* USBD_CDC_H */
