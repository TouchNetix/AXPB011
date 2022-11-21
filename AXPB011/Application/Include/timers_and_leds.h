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
#ifndef APPLICATION_INCLUDE_TIMERS_AND_LEDS_H_
#define APPLICATION_INCLUDE_TIMERS_AND_LEDS_H_

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
#define DIGITIZER_TIMER         (TIMER1)
#define PROXY_DELAY_TIMER       (TIMER13)

#define AXIOM_COMMS_DETECTED    (1U)
#define NO_AXIOM_COMMS          (0U)
#define USB_COMMS_DETECTED      (1U)
#define NO_USB_COMMS            (0U)

#define LED_USB_GPIO_PORT       (GPIOB)
#define LED_USB_GPIO_PIN        (GPIO_PIN_0)
#define LED_AXIOM_GPIO_PORT     (GPIOB)
#define LED_AXIOM_GPIO_PIN      (GPIO_PIN_1)

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

 
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
void TimerInit(void);
void LEDsInit(void);
void StartProxyDelayTimer(void);
void ControlLEDs(void);
void NotifyUSBComms(void);
void NotifyAxiomComms(void);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* APPLICATION_INCLUDE_TIMERS_AND_LEDS_H_ */

