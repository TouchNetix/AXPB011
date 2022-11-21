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
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "usb_conf.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * Data Types
 ******************************************************************************/


/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define USBD_CFG_MAX_NUM    (  1U)
#define USBD_ITF_MAX_NUM    (  3U)

#define USB_COMPOSITE_CONFIG_DESC_LEN_NO_DIGITIZER  (73U)
#define USB_COMPOSITE_CONFIG_DESC_LEN               (98U)

#define USB_STR_DESC_MAX_SIZE               (255U)

/*---------- GENERIC -----------*/

#define GENERIC_INTERFACE_NUM               (  0U)
#define GENERIC_HID_EP_NUM                  (  1U)
#define GENERIC_HID_IN_EP                   (EP_IN(GENERIC_HID_EP_NUM))
#define GENERIC_HID_OUT_EP                  (EP_OUT(GENERIC_HID_EP_NUM))
#define GENERIC_HID_OUT_PACKET_SIZE         ( 64U)
#define GENERIC_IN_PACKET_SIZE              ( 64U)
#define GENERIC_HID_FEATURE_PACKET_SIZE     (  4U)

/*---------- PRESS -----------*/

#define PRESS_INTERFACE_NUM                 (  1U)
#define PRESS_HID_EP_NUM                    (  2U)
#define PRESS_HID_IN_EP                     (EP_IN(PRESS_HID_EP_NUM))
#define PRESS_HID_OUT_EP                    (EP_OUT(PRESS_HID_EP_NUM))
#define PRESS_IN_PACKET_SIZE                ( 64U)
#define PRESS_HID_OUT_PACKET_SIZE           ( 64U)
#define PRESS_HID_FEATURE_PACKET_SIZE       (  4U)

/*---------- DIGITIZER -----------*/

#define DIGITIZER_INTERFACE_NUM             (  2U)
#define DIGITIZER_HID_EP_NUM                (  3U)
#define DIGITIZER_HID_IN_EP                 (EP_IN(DIGITIZER_HID_EP_NUM))
#define DIGITIZER_HID_OUT_EP                (EP_OUT(DIGITIZER_HID_EP_NUM))
#define DIGITIZER_IN_PACKET_SIZE            ( 39U)
#define MOUSE_ABS_IN_PACKET_SIZE            (  5U)

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/


/*******************************************************************************
 * Exported Functions
 ******************************************************************************/


/*******************************************************************************
 * Macros
 ******************************************************************************/


/*******************************************************************************
 * Inline Functions
 ******************************************************************************/

/* END SENTRY GUARD */
#endif /* __USBD_CONF_H */
