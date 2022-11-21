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
#include "press_hid_core.h"
#include "usbd_enum.h"
#include "timers_and_leds.h"
#include <string.h>

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
uint8_t PressCommandReceived = 0;

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define USBD_VID                     0x28E9U
#define USBD_PID                     0x028AU

/* USB Report Descriptor */
const uint8_t press_report_descriptor[PRESS_REPORT_DESC_LEN] =
{
            // item code|no. bytes
    0x06, 0xFF, 0xFF,   // 04|2   , Usage Page (vendor defined?)
    0x09, 0x01,         // 08|1   , Usage      (vendor defined
    0xA1, 0x01,         // A0|1   , Collection (Application)
    /* 7 bytes */

    // IN report
    0x09, 0x02,         // 08|1   , Usage      (vendor defined)
    0x09, 0x03,         // 08|1   , Usage      (vendor defined)
    0x15, 0x00,         // 14|1   , Logical Minimum(0 for signed byte?)
    0x26, 0xFF, 0x00,   // 24|1   , Logical Maximum(255 for signed byte?)
    0x75, 0x08,         // 74|1   , Report Size(8) = field size in bits = 1 byte
    0x95, PRESS_IN_PACKET_SIZE,   // 94|2 ReportCount(size) = repeat count of previous item, 64 byte IN report
    0x81, 0x02,         // 80|1   , IN report (Data,Variable, Absolute)
    /* 22 bytes */

    // OUT report
    0x09, 0x04,         // 08|1   , Usage      (vendor defined)
    0x09, 0x05,         // 08|1   , Usage      (vendor defined)
    0x15, 0x00,         // 14|1   , Logical Minimum(0 for signed byte?)
    0x26, 0xFF, 0x00,   // 24|1   , Logical Maximum(255 for signed byte?)
    0x75, 0x08,         // 74|1   , Report Size(8) = field size in bits = 1 byte
    0x95, PRESS_HID_OUT_PACKET_SIZE,   // 94|2 ReportCount(size) = repeat count of previous item, 64 byte OUT report
    0x91, 0x02,         // 90|1   , OUT report (Data,Variable, Absolute)
    /* 37 bytes */

    // Feature report
    0x09, 0x06,         // 08|1   , Usage      (vendor defined)
    0x09, 0x07,         // 08|1   , Usage      (vendor defined)
    0x15, 0x00,         // 14|1   , LogicalMinimum(0 for signed byte)
    0x26, 0xFF, 0x00,   // 24|1   , Logical Maximum(255 for signed byte)
    0x75, 0x08,         // 74|1   , Report Size(8) = field size in bits = 1 byte
    0x95, PRESS_HID_FEATURE_PACKET_SIZE,         // 94|1   , ReportCount in byte
    0xB1, 0x02,         // B0|1   , Feature report
    0xC0                // C0|0   , End Collection
    /* 53 bytes */
};

/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/


/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/
#define UNUSED(x) ((void)(x))

/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static uint8_t press_hid_init(usb_dev *udev, uint8_t config_index, usb_desc_ep *epin, usb_desc_ep *epout);
static uint8_t press_hid_deinit(usb_dev *udev, uint8_t config_index);
static uint8_t press_hid_req_handler(usb_dev *udev, usb_req *req);
static uint8_t press_hid_data_in(usb_dev *udev, uint8_t ep_num);
static uint8_t press_hid_data_out(usb_dev *udev, uint8_t ep_num);

usb_custom_class_core usbd_press_hid_cb = {
    .command   = NO_CMD,
    .alter_set = 0U,

    .init      = press_hid_init,
    .deinit    = press_hid_deinit,

    .req_proc  = press_hid_req_handler,

    .data_in   = press_hid_data_in,
    .data_out  = press_hid_data_out
};

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/*!
    \brief      register HID interface operation functions
    \param[in]  udev: pointer to USB device instance
    \param[in]  hid_fop: HID operation functions structure
    \param[out] none
    \retval     USB device operation status
*/
uint8_t press_hid_itfop_register(usb_dev *udev, press_fop_handler *hid_fop)
{
    if (NULL != hid_fop)
    {
        udev->dev.user_data[PRESS_INTERFACE_NUM] = hid_fop;

        return USBD_OK;
    }

    return USBD_FAIL;
}

/*!
    \brief      send press HID report
    \param[in]  udev: pointer to USB device instance
    \param[in]  report: pointer to HID report
    \param[in]  len: data length
    \param[out] none
    \retval     USB device operation status
*/
uint8_t press_hid_report_send(usb_dev *udev, uint8_t *report, uint32_t len)
{
    usbd_status status = USBD_BUSY;

    press_hid_handler *hhid = (press_hid_handler *)udev->dev.class_data[PRESS_INTERFACE_NUM];

    if (udev->dev.cur_status == USBD_CONFIGURED)
    {
        if (hhid->state == eIDLE)
        {
            hhid->state = eBUSY;
            usbd_ep_send(udev, PRESS_HID_IN_EP, report, len);
            NotifyUSBComms();
            status = USBD_OK;
        }
    }

    return status;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
/*!
    \brief      initialize the HID device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t press_hid_init(usb_dev *udev, uint8_t config_index, usb_desc_ep *epin, usb_desc_ep *epout)
{
    static press_hid_handler hid_handler;

    memset((void *)&hid_handler, 0U, sizeof(press_hid_handler));

    /* Initialize the data Tx endpoint */
    usbd_ep_setup(udev, epin);
    usbd_ep_setup(udev, epout);

    /* prepare receive data */
    usbd_ep_recev(udev, PRESS_HID_OUT_EP, hid_handler.data, PRESS_HID_OUT_PACKET_SIZE);

    udev->dev.class_data[PRESS_INTERFACE_NUM] = (void *)&hid_handler;

    if (udev->dev.user_data[PRESS_INTERFACE_NUM] != NULL)
    {
        if (((press_fop_handler*) udev->dev.user_data[PRESS_INTERFACE_NUM])->periph_config != NULL)
        {
            ((press_fop_handler*) udev->dev.user_data[PRESS_INTERFACE_NUM])->periph_config();
        }
    }

    return USBD_OK;
}

/*!
    \brief      de-initialize the HID device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t press_hid_deinit(usb_dev *udev, uint8_t config_index)
{
    /* deinitialize HID endpoints */
    usbd_ep_clear(udev, PRESS_HID_IN_EP);

    return USBD_OK;
}

/*!
    \brief      handle the HID class-specific requests
    \param[in]  udev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t press_hid_req_handler(usb_dev *udev, usb_req *req)
{
    usb_transc *transc = &udev->dev.transc_in[0];

    press_hid_handler *hid = (press_hid_handler *)udev->dev.class_data[PRESS_INTERFACE_NUM];

    switch(req->bRequest) {
    case GET_REPORT:
        break;

    case GET_IDLE:
        transc->xfer_buf = (uint8_t *)&hid->idlestate;
        transc->remain_len = 1U;
        break;

    case GET_PROTOCOL:
        transc->xfer_buf = (uint8_t *)&hid->protocol;
        transc->remain_len = 1U;
        break;

    case SET_REPORT:
        hid->reportID = (uint8_t)(req->wValue);
        break;

    case SET_IDLE:
        hid->idlestate = (uint8_t)(req->wValue >> 8U);
        break;

    case SET_PROTOCOL:
        hid->protocol = (uint8_t)(req->wValue);
        break;

    case USB_GET_DESCRIPTOR:
        if(USB_DESCTYPE_REPORT == (req->wValue >> 8U)) {
            transc->remain_len = USB_MIN(PRESS_REPORT_DESC_LEN, req->wLength);
            transc->xfer_buf = (uint8_t *)press_report_descriptor;
        }
        break;

    default:
        return USBD_FAIL;
    }

    return USBD_OK;
}

/*!
    \brief      handle press HID data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t press_hid_data_in(usb_dev *udev, uint8_t ep_num)
{
    press_hid_handler *hhid = (press_hid_handler *)udev->dev.class_data[PRESS_INTERFACE_NUM];
    hhid->state = eIDLE;

    return USBD_OK;
}

/*!
    \brief      handle press HID data
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t press_hid_data_out(usb_dev *udev, uint8_t ep_num)
{
    press_hid_handler *hid = (press_hid_handler *)udev->dev.class_data[PRESS_INTERFACE_NUM];

    usbd_ep_recev(udev, PRESS_HID_OUT_EP, hid->data, PRESS_HID_OUT_PACKET_SIZE);

    ((press_fop_handler*)udev->dev.user_data[PRESS_INTERFACE_NUM])->PressCommandReceived = PRESS_COMMAND_RECEIVED;

    return USBD_OK;
}
