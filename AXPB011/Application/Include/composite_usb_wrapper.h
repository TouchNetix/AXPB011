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

/* Sentry Guard Start */
#ifndef HID_PRINTER_WRAPPER_H
#define HID_PRINTER_WRAPPER_H

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "usb_ch9_std.h"
#include "usb_hid.h"
#include "drv_usb_dev.h"
#include "generic_hid_itf.h"
#include "press_hid_itf.h"
#include "digitizer_hid_itf.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * Data Types
 ******************************************************************************/
typedef struct {
    usb_desc_config                config;

    usb_desc_itf                   generic_interface;
    usb_desc_hid                   generic_hid_descriptor;
    usb_desc_ep                    generic_epin;
    usb_desc_ep                    generic_epout;

    usb_desc_itf                   press_interface;
    usb_desc_hid                   press_hid_descriptor;
    usb_desc_ep                    press_epin;
    usb_desc_ep                    press_epout;

    usb_desc_itf                   digitizer_interface;
    usb_desc_hid                   digitizer_hid_descriptor;
    usb_desc_ep                    digitizer_epin;
} usb_composite_desc_config_set;

/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
extern usb_desc_dev composite_dev_desc;
extern usb_composite_desc_config_set composite_config_desc;
extern usb_desc hid_composite_desc;
extern usb_class_core usbd_hid_composite_cb;

/*******************************************************************************
 * Macros
 ******************************************************************************/


/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* HID_PRINTER_WRAPPER_H */
