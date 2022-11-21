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

#ifndef APPLICATION_INCLUDE_COMMAND_PROCESSOR_H_
#define APPLICATION_INCLUDE_COMMAND_PROCESSOR_H_

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "generic_hid_core.h"
#include "press_hid_core.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/

/*******************************************************************************
 * Data Types
 ******************************************************************************/


/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define CONTROL_INTERFACE   (0)
#define PRESS_INTERFACE     (1U)

#define RESPONSE_REQUIRED       (1U)
#define NO_RESPONSE_REQUIRED    (0)

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

 
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
uint8_t ProcessTBPCommand(usb_core_driver *udev, uint8_t *pbuf, uint8_t which_interface);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* APPLICATION_INCLUDE_COMMAND_PROCESSOR_H_ */
