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
// USB feature -- Self Powered
// #define USBD_SELF_POWERED

// USB user string supported
// #define USB_SUPPORT_USER_STRING_DESC

//#define USBD_DYNAMIC_DESCRIPTOR_CHANGE_ENABLED

/*******************************************************************************
 * Data Types
 ******************************************************************************/


/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define USBD_CFG_MAX_NUM              1U
#define USBD_ITF_MAX_NUM              1U
#define USB_STR_DESC_MAX_SIZE         64U

#define USBD_DFU_INTERFACE            0U

/* Maximum number of supported media (Flash) */
#define MAX_USED_MEMORY_MEDIA        1U

#define USB_STRING_COUNT             6U

/* DFU maximum data packet size */
#define TRANSFER_SIZE                2048U

/* memory address from where user application will be loaded, which represents
   the dfu code protected against write and erase operations.*/
#define APP_LOADED_ADDR              0x08004000U

/* DFU endpoint define */
#define DFU_IN_EP                    EP0_IN
#define DFU_OUT_EP                   EP0_OUT

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/


/*******************************************************************************
 * Exported Functions
 ******************************************************************************/


/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Make sure the corresponding memory where the DFU code should not be loaded
   cannot be erased or overwritten by DFU application. */
#define IS_PROTECTED_AREA(addr)      (uint8_t)(((addr >= 0x08000000U) && (addr < (APP_LOADED_ADDR)))? 1U : 0U)

/*******************************************************************************
 * Inline Functions
 ******************************************************************************/

/* END SENTRY GUARD */
#endif /* __USBD_CONF_H */
