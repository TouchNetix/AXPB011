/*!
    @file    composite_usb_wrapper.c
    @brief   This file calls to the separate HID classes layer handlers.

    @version 2020-08-13, V1.0, firmware for GD32F3x0
*/

/*******************************************************************************
*                                    NOTICE
*
* Copyright (c) 2010 - 2022 TouchNetix Limited
* ALL RIGHTS RESERVED.
*
* The source  code contained  or described herein  and all documents  related to
* the source code ("Material") are owned by TouchNetix Limited ("TouchNetix") or
* its suppliers  or licensors. Title to the Material  remains with TouchNetix or
* its   suppliers  and  licensors. The  Material  contains  trade  secrets   and
* proprietary  and confidential information  of TouchNetix or its  suppliers and
* licensors.  The  Material  is  protected  by  worldwide  copyright  and  trade
* secret  laws and  treaty  provisions.  No part  of the Material  may be  used,
* copied,  reproduced,  modified,   published,  uploaded,  posted,  transmitted,
* distributed or disclosed in any way without TouchNetix's prior express written
* permission.
*
* No  license under any  patent, copyright,  trade secret or other  intellectual
* property  right is granted to or conferred upon you by disclosure or  delivery
* of  the Materials, either  expressly, by implication, inducement, estoppel  or
* otherwise.  Any  license  under  such  intellectual  property rights  must  be
* expressly approved by TouchNetix in writing.
*
*******************************************************************************/

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "composite_usb_wrapper.h"
#include "generic_hid_core.h"
#include "press_hid_core.h"
#include "digitizer_hid_core.h"
#include "mode_control.h"

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/


/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define USBD_VID                (0x28E9U)   // Gigadevice VID
#define USBD_PID                (0x0000U)   // Updated during init

#define DEVICE_FW_VERSION_MAJOR (0x01U)
#define DEVICE_FW_VERSION_MINOR (0x06U)

#define DEVICE_FW_VERSION       ((uint16_t)((DEVICE_FW_VERSION_MAJOR << 8U) | (DEVICE_FW_VERSION_MINOR)))

#define USB_ATTR_RESERVED       (1U << 7U)
#define USB_ATTR_BUS_POWERED    (0U << 6U)
#define USB_ATTR_REMOTE_WAKEUP  (1U << 5U)


/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
// USB device descriptor
// NOTE: This cannot be const - it is edited depending on the mode selected.
__ALIGN_BEGIN usb_desc_dev composite_dev_desc __ALIGN_END = {
    .header =
    {
        .bLength          = USB_DEV_DESC_LEN,
        .bDescriptorType  = USB_DESCTYPE_DEV,
    },
    .bcdUSB                = 0x0200,
    .bDeviceClass          = 0x00,
    .bDeviceSubClass       = 0x00,
    .bDeviceProtocol       = 0x00,
    .bMaxPacketSize0       = USB_FS_EP0_MAX_LEN,
    .idVendor              = USBD_VID,
    .idProduct             = USBD_PID,
    .bcdDevice             = DEVICE_FW_VERSION, //DEVICE_FW_VERSION
    .iManufacturer         = STR_IDX_MFC,
    .iProduct              = STR_IDX_PRODUCT,
    .iSerialNumber         = STR_IDX_SERIAL,
    .bNumberConfigurations = USBD_CFG_MAX_NUM,
};

// USB configuration descriptor
// NOTE: This cannot be const - it is edited depending on the mode selected
__ALIGN_BEGIN usb_composite_desc_config_set composite_config_desc __ALIGN_END = {
    .config =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_config),
            .bDescriptorType = USB_DESCTYPE_CONFIG,
        },
        .wTotalLength         = sizeof(usb_composite_desc_config_set),
        .bNumInterfaces       = 0x00, // Set depending on bridge mode
        .bConfigurationValue  = 0x01,
        .iConfiguration       = 0x00,
        .bmAttributes         = USB_ATTR_RESERVED | USB_ATTR_BUS_POWERED | USB_ATTR_REMOTE_WAKEUP,
        .bMaxPower            = 0xC8    /* 400mA */
    },

    .generic_interface =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber     = GENERIC_INTERFACE_NUM,
        .bAlternateSetting    = 0x00,
        .bNumEndpoints        = 0x02,
        .bInterfaceClass      = 0x03,   /* HID */
        .bInterfaceSubClass   = 0x00,
        .bInterfaceProtocol   = 0x00,
        .iInterface           = STR_IDX_ITF_GENERIC
    },

    .generic_hid_descriptor =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_hid),
            .bDescriptorType = USB_DESCTYPE_HID
        },
        .bcdHID               = 0x0111,
        .bCountryCode         = 0x00,
        .bNumDescriptors      = 0x01,
        .bDescriptorType      = USB_DESCTYPE_REPORT,
        .wDescriptorLength    = GENERIC_REPORT_DESC_LEN,
    },

    .generic_epin =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress     = GENERIC_HID_IN_EP,
        .bmAttributes         = USB_EP_ATTR_INT,
        .wMaxPacketSize       = GENERIC_IN_PACKET_SIZE,
        .bInterval            = 0x01    /* Polling interval in ms */
    },

    .generic_epout =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress     = GENERIC_HID_OUT_EP,
        .bmAttributes         = USB_EP_ATTR_INT,
        .wMaxPacketSize       = GENERIC_HID_OUT_PACKET_SIZE,
        .bInterval            = 0x01    /* Polling interval in ms */
    },

    .press_interface =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber     = PRESS_INTERFACE_NUM,
        .bAlternateSetting    = 0x00,
        .bNumEndpoints        = 0x02,
        .bInterfaceClass      = 0x03,   /* HID */
        .bInterfaceSubClass   = 0x00,
        .bInterfaceProtocol   = 0x00,
        .iInterface           = STR_IDX_ITF_PRESS
    },

    .press_hid_descriptor =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_hid),
            .bDescriptorType = USB_DESCTYPE_HID
        },
        .bcdHID               = 0x0111,
        .bCountryCode         = 0x00,
        .bNumDescriptors      = 0x01,
        .bDescriptorType      = USB_DESCTYPE_REPORT,
        .wDescriptorLength    = PRESS_REPORT_DESC_LEN,
    },

    .press_epin =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress     = PRESS_HID_IN_EP,
        .bmAttributes         = USB_EP_ATTR_INT,
        .wMaxPacketSize       = PRESS_IN_PACKET_SIZE,
        .bInterval            = 0x01    /* Polling interval in ms */
    },

    .press_epout =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress     = PRESS_HID_OUT_EP,
        .bmAttributes         = USB_EP_ATTR_INT,
        .wMaxPacketSize       = PRESS_HID_OUT_PACKET_SIZE,
        .bInterval            = 0x01    /* Polling interval in ms */
    },

    .digitizer_interface =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber     = DIGITIZER_INTERFACE_NUM,
        .bAlternateSetting    = 0x00,
        .bNumEndpoints        = 0x01,
        .bInterfaceClass      = 0x03,   /* HID */
        .bInterfaceSubClass   = 0x00,
        .bInterfaceProtocol   = 0x00,
        .iInterface           = STR_IDX_ITF_DIGITIZER
    },

    .digitizer_hid_descriptor =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_hid),
            .bDescriptorType = USB_DESCTYPE_HID
        },
        .bcdHID               = 0x0111,
        .bCountryCode         = 0x00,
        .bNumDescriptors      = 0x01,
        .bDescriptorType      = USB_DESCTYPE_REPORT,
        .wDescriptorLength    = DIGITIZER_REPORT_DESC_LEN,
    },

    .digitizer_epin =
    {
            .header =
            {
                .bLength         = sizeof(usb_desc_ep),
                .bDescriptorType = USB_DESCTYPE_EP
            },
            .bEndpointAddress     = DIGITIZER_HID_IN_EP,
            .bmAttributes         = USB_EP_ATTR_INT,
            .wMaxPacketSize       = 0x00,   // Updated during initialisation depending on digitizer mode
            .bInterval            = 0x01    /* Polling interval in ms */
    }
};

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/

// Note:it should use the C99 standard when compiling the below codes.

/* USB language ID Descriptor */
static __ALIGN_BEGIN const usb_desc_LANGID usbd_language_id_desc __ALIGN_END = {
    .header =
    {
        .bLength         = sizeof(usb_desc_LANGID),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .wLANGID              = ENG_LANGID
};

/* USB manufacturer string */
static __ALIGN_BEGIN const usb_desc_str manufacturer_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(10),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'T', 'o', 'u', 'c', 'h', 'N', 'e', 't', 'i', 'x'}
};

/* USB product string */
static __ALIGN_BEGIN const usb_desc_str product_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(7),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'A','X','P','B','0','1','1'}
};

/* USB serial string */
static __ALIGN_BEGIN usb_desc_str serial_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(12),
        .bDescriptorType = USB_DESCTYPE_STR,
    }
};

/* Generic interface string */
static __ALIGN_BEGIN usb_desc_str generic_itf_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(15),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'A','X','P','B','0','1','1', ' ', 'C', 'o', 'n', 't', 'r', 'o', 'l' }
};

/* Press interface string */
static __ALIGN_BEGIN usb_desc_str press_itf_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(18),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'A','X','P','B','0','1','1', ' ', 'P', 'r', 'e', 's', 's', ' ', 'D', 'a', 't', 'a'}
};

/* Digitizer interface string */
static __ALIGN_BEGIN usb_desc_str digitizer_itf_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(17),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'A','X','P','B','0','1','1', ' ', 'D', 'i', 'g', 'i', 't', 'i', 'z' , 'e', 'r'}
};

/* Mouse interface string */
static __ALIGN_BEGIN usb_desc_str mouse_itf_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(17),
        .bDescriptorType = USB_DESCTYPE_STR,
    },
    .unicode_string = {'A','X','P','B','0','1','1', ' ', 'A', 'b', 's', ' ', 'M', 'o', 'u' , 's', 'e'}
};

/* USB string descriptor set */
void *const usb_device_strings[] = {
    [STR_IDX_LANGID]        = (uint8_t *) &usbd_language_id_desc,
    [STR_IDX_MFC]           = (uint8_t *) &manufacturer_string,
    [STR_IDX_PRODUCT]       = (uint8_t *) &product_string,
    [STR_IDX_SERIAL]        = (uint8_t *) &serial_string,
    [STR_IDX_ITF_GENERIC]   = (uint8_t *) &generic_itf_string,
    [STR_IDX_ITF_PRESS]     = (uint8_t *) &press_itf_string,
    [STR_IDX_ITF_DIGITIZER] = (uint8_t *) &digitizer_itf_string,
    [STR_IDX_ITF_MOUSE]     = (uint8_t *) &mouse_itf_string,
};

usb_desc hid_composite_desc = {
    .dev_desc    = (uint8_t *) &composite_dev_desc,
    .config_desc = (uint8_t *) &composite_config_desc,
    .strings     = usb_device_strings
};

/* local function prototypes ('static') */
static uint8_t hid_composite_init(usb_dev *udev, uint8_t config_index);
static uint8_t hid_composite_deinit(usb_dev *udev, uint8_t config_index);
static uint8_t hid_composite_req_handler(usb_dev *udev, usb_req *req);
static uint8_t hid_composite_data_in(usb_dev *udev, uint8_t ep_num);
static uint8_t hid_composite_data_out(usb_dev *udev, uint8_t ep_num);

usb_class_core usbd_hid_composite_cb = {
    .init      = hid_composite_init,
    .deinit    = hid_composite_deinit,
    .req_proc  = hid_composite_req_handler,
    .data_in   = hid_composite_data_in,
    .data_out  = hid_composite_data_out,
};

/*!
    \brief      initialize the HID/printer device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t hid_composite_init(usb_dev *udev, uint8_t config_index)
{
    /* HID initialization */
    usbd_generic_hid_cb.init(udev, config_index, &composite_config_desc.generic_epin, &composite_config_desc.generic_epout);
    usbd_press_hid_cb.init(udev, config_index, &composite_config_desc.press_epin, &composite_config_desc.press_epout);

    // TODO JC - reading the flash seems to be leading to a hardfault.
    uint8_t mode = GetBridgeMode();
    if ((mode == MODE_ABSOLUTE_MOUSE) || (mode == MODE_PARALLEL_DIGITIZER))
    {
        usbd_digitizer_hid_cb.init(udev, config_index, &composite_config_desc.digitizer_epin, NULL);
    }

    return USBD_OK;
}

/*!
    \brief      de-initialize the HID/printer device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t hid_composite_deinit(usb_dev *udev, uint8_t config_index)
{
    /* HID De-initialization */
    usbd_generic_hid_cb.deinit(udev, config_index);
    usbd_press_hid_cb.deinit(udev, config_index);
    usbd_digitizer_hid_cb.deinit(udev, config_index);

    return USBD_OK;
}

/*!
    \brief      handle the composite HID/printer class-specific request
    \param[in]  udev: pointer to USB device instance
    \param[in]  req: device class request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t hid_composite_req_handler(usb_dev *udev, usb_req *req)
{
    if ((req->wIndex & 0xFF) == PRESS_INTERFACE_NUM)
    {
        return usbd_press_hid_cb.req_proc(udev, req);
    }
    else if ((req->wIndex & 0xFF) == DIGITIZER_INTERFACE_NUM)
    {
        return usbd_digitizer_hid_cb.req_proc(udev, req);
    }
    else //if ((req->wIndex & 0xFF) == GENERIC_INTERFACE_NUM)
    {
        return usbd_generic_hid_cb.req_proc(udev, req);
    }
}

/*!
    \brief      handle data IN stage
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint number
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t hid_composite_data_in(usb_dev *udev, uint8_t ep_num)
{
    if ((PRESS_HID_IN_EP & 0x7F) == ep_num)
    {
        return usbd_press_hid_cb.data_in(udev, ep_num);
    }
    else if ((DIGITIZER_HID_IN_EP & 0x7F) == ep_num)
    {
        return usbd_digitizer_hid_cb.data_in(udev, ep_num);
    }
    else
    {
        return usbd_generic_hid_cb.data_in(udev, ep_num);
    }
}

/*!
    \brief      handle data OUT stage
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint number
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t hid_composite_data_out(usb_dev *udev, uint8_t ep_num)
{
    if ((PRESS_HID_OUT_EP & 0x7F) == ep_num)
    {
        return usbd_press_hid_cb.data_out(udev, ep_num);
    }
    else if ((DIGITIZER_HID_OUT_EP & 0x7F) == ep_num)
    {
        return usbd_digitizer_hid_cb.data_out(udev, ep_num);
    }
    else
    {
        return usbd_generic_hid_cb.data_out(udev, ep_num);
    }
}
