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
#ifndef APPLICATION_INCLUDE_DIGITIZER_HID_ITF_H_
#define APPLICATION_INCLUDE_DIGITIZER_HID_ITF_H_

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "gd32f3x0.h"
#include "usbd_enum.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * Data Types
 ******************************************************************************/
typedef enum
{
    eDigitizerPacketsBlocked = 0,
    eDigitizerPacketsAllowed = 1U,
} enDigitizerBlockedStatus;

typedef enum
{
    eNoWake                 = 0x00U,
    eWakeOnTouch            = 0x01U,
    eWakeOnTouchHover       = 0x03U,
    eWakeOnTouchHoverProx   = 0x07U,
} en_HostWakeOptions;

/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define DIGITIZER_USB_PACKET_READY  (1U)
#define NO_DIGITIZER_USB_PACKET     (0)

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

 
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
void PrepareDigitizerUSBReport(usb_dev *udev, uint8_t *pReport, uint8_t *pMsg);
void IncrememtDigitizerTimeStamp(void);
uint8_t CheckForDigitizerUSBPacket(void);
void ClearDigitizerUSBPacketReady(void);
void BlockDigitizerPackets(enDigitizerBlockedStatus state);
uint32_t GetDigitizerReportLength(void);
void SetHostWakeOption(en_HostWakeOptions wake_option);
en_HostWakeOptions GetHostWakeOption(void);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* APPLICATION_INCLUDE_DIGITIZER_HID_ITF_H_ */
