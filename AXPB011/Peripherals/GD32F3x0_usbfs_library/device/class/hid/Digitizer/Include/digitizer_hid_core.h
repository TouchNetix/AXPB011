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
#ifndef __DIGITIZER_HID_CORE_H_
#define __DIGITIZER_HID_CORE_H_

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "usbd_enum.h"
#include "usb_hid.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * Data Types
 ******************************************************************************/


/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define MAX_NUM_DIGITIZER_TOUCHES   (  5U)
#define ABS_MOUSE_REPORT_DESC_LEN   ( 58U)
#define DIGITIZER_REPORT_DESC_LEN   (464U)
#define NO_CMD                      (0xFFU)

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
typedef struct
{
    uint8_t         reportID;
    uint8_t         idlestate;
    uint8_t         protocol;
    enUSBItfState   state;
} digitizer_hid_handler;

typedef struct
{
    void (*periph_config)(void);
} digitizer_fop_handler;

typedef struct {
    usb_desc_ep digitizer_epin;
    usb_desc_ep digitizer_epout;
} usb_digitizer_desc_config_set;

extern usb_desc         		digitizer_hid_desc;
extern usb_custom_class_core	usbd_digitizer_hid_cb;

extern uint8_t digitizer_report_descriptor[DIGITIZER_REPORT_DESC_LEN];
extern const usb_digitizer_desc_config_set digitizer_hid_config_desc;
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
uint8_t digitizer_hid_itfop_register(usb_dev *udev, digitizer_fop_handler *hid_fop);
uint8_t digitizer_hid_report_send(usb_dev *udev, uint8_t *report, uint32_t len);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* digitizer_HID_CORE_H_ */

