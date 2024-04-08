#pragma once

#include "usbd_core.h"
#include "usbd_video.h"

#define VIDEO_IN_EP 0x81

#ifdef CONFIG_USB_HS
#define MAX_PAYLOAD_SIZE  1024 // for high speed with one transcations every one micro frame
#define VIDEO_PACKET_SIZE (unsigned int)(((MAX_PAYLOAD_SIZE / 1)) | (0x00 << 11))

// #define MAX_PAYLOAD_SIZE  2048 // for high speed with two transcations every one micro frame
// #define VIDEO_PACKET_SIZE (unsigned int)(((MAX_PAYLOAD_SIZE / 2)) | (0x01 << 11))

// #define MAX_PAYLOAD_SIZE  3072 // for high speed with three transcations every one micro frame
// #define VIDEO_PACKET_SIZE (unsigned int)(((MAX_PAYLOAD_SIZE / 3)) | (0x02 << 11))

#else
#define MAX_PAYLOAD_SIZE  1020
#define VIDEO_PACKET_SIZE (unsigned int)(((MAX_PAYLOAD_SIZE / 1)) | (0x00 << 11))
#endif

#define WIDTH  (unsigned int)(640)
#define HEIGHT (unsigned int)(480)

#define CAM_FPS        (30)
#define INTERVAL       (unsigned long)(10000000 / CAM_FPS)
#define MIN_BIT_RATE   (unsigned long)(WIDTH * HEIGHT * 16 * CAM_FPS) //16 bit
#define MAX_BIT_RATE   (unsigned long)(WIDTH * HEIGHT * 16 * CAM_FPS)
#define MAX_FRAME_SIZE (unsigned long)(WIDTH * HEIGHT * 2)

#define USB_VIDEO_DESC_SIZ (unsigned long)(9 + 8 + 9 + 13 + 18 +9 + 12 + 9 + 14 + 11 + 30 + 9 + 7)

#define VC_TERMINAL_SIZ (unsigned int)(13 + 18 + 12 + 9)
#define VS_HEADER_SIZ   (unsigned int)(13 + 1 + 11 + 30)

#define USBD_VID           0x20B1
#define USBD_PID           0x1DE0
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

uint8_t video_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x01, USBD_VID, USBD_PID, 0x0001, 0x01),
    /*Configuration Descriptor*/
    0x09,       /* bLength: Configuration Descriptor size */
    0x02,      /* bDescriptorType: Configuration */
    41,                /* wTotalLength:no of returned bytes */
    0x00,
    0x02,   /* bNumInterfaces: 1 interface for Game IO */
    0x01,   /* bConfigurationValue: Configuration value */
    0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
    0xE0,   /* bmAttributes: self powered */
    0x32,   /* MaxPower 0 mA */
    
    /*---------------------------------------------------------------------------*/
    
    /*Data class interface descriptor*/
    0x09,   /* bLength: Endpoint Descriptor size */
    0x04,  /* bDescriptorType: */
    0x00,   /* bInterfaceNumber: Number of Interface, zero based index of this interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: Two endpoints used */
    0x00,   /* bInterfaceClass: vendor */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */
    
    /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    0x05,                              /* bDescriptorType: Endpoint */
    0x81,                               /* bEndpointAddress */
    0x02,                               /* bmAttributes: Bulk */
    0x40,                               /* wMaxPacketSize: */
    0x00,
    0x00,                                /* bInterval: ignore for Bulk transfer */

    
    /*Data class interface descriptor*/
    0x09,   /* bLength: Endpoint Descriptor size */
    0x04,  /* bDescriptorType: */
    0x01,   /* bInterfaceNumber: Number of Interface, zero based index of this interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: Two endpoints used */
    0x00,   /* bInterfaceClass: vendor */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */

    /*Endpoint OUT Descriptor*/
    0x07,                               /* bLength: Endpoint Descriptor size */
    0x05,                               /* bDescriptorType: Endpoint */
    0x02,                               /* bEndpointAddress */
    0x02,                               /* bmAttributes: Control 0x0 Isochronous 0x1 Bulk 0x2 Interrupt 0x3*/
    0x40,                               /* wMaxPacketSize: */
    0x00,
    0x00,                                /* bInterval: ignore for Bulk transfer */

    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'x', 0x00,                  /* wcChar0 */
    'e', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'c', 0x00,                  /* wcChar3 */
    'o', 0x00,                  /* wcChar4 */
    's', 0x00,                  /* wcChar5 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'E', 0x00,                  /* wcChar0 */
    'z', 0x00,                  /* wcChar1 */
    'C', 0x00,                  /* wcChar2 */
    'a', 0x00,                  /* wcChar3 */
    'm', 0x00,                  /* wcChar4 */
    'e', 0x00,                  /* wcChar5 */
    'r', 0x00,                  /* wcChar6 */
    'a', 0x00,                  /* wcChar7 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '1', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    '1', 0x00,                  /* wcChar6 */
    '0', 0x00,                  /* wcChar7 */
    '0', 0x00,                  /* wcChar8 */
    '0', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00,
    // 0xa,0xa,0xa,0xa
};