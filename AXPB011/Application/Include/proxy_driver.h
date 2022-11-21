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
#ifndef APPLICATION_INCLUDE_PROXY_DRIVER_H_
#define APPLICATION_INCLUDE_PROXY_DRIVER_H_

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "gd32f3x0.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * Data Types
 ******************************************************************************/


/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define MAX_AXIOM_REPORT_LEN    (62U) // Largest possible report that can be read from axiom
#define U34_PROXYLENGTH         (64U)
#define PROXY_FLAG              (0x9AU)

typedef enum
{
    eReportRead = 0U,
    eNoReport   = 1U,
} enProxyStatus;

typedef enum
{
    eDataRead = 0U,
    eLastRead = 1U,
    eError    = 2U,
} enMultiPageStatus;

typedef enum
{
    ePressDigiProxy = 0U,
    eControlPressDigiProxy = 1U,
    eMultiPageRead = 2U,
    eProxyStop = 3U,
} enBridgeProxyMode;

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

 
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
void SetProxyMode(enBridgeProxyMode NewMode);
enBridgeProxyMode GetCurrentProxyMode(void);
enProxyStatus ReadProxyReport(uint8_t *pbuf);
enMultiPageStatus MultiPageProxyReport(uint8_t *pbuf);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* APPLICATION_INCLUDE_PROXY_DRIVER_H_ */

