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
#ifndef APPLICATION_INCLUDE_COMMS_H_
#define APPLICATION_INCLUDE_COMMS_H_

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
#define AXIOMCOMMS_READWRITE_OK     (0x00U)
#define AXIOMCOMMS_COMMS_FAILED     (0x01U)
#define AXIOMCOMMS_DEVICE_TIMEOUT   (0x02U)
#define AXIOMCOMMS_WRITE_OK_NOREAD  (0x04U)
#define AXIOMCOMMS_INCORRECTSETUP   (0xFFU)

#define NIRQ_GPIO_PORT      (GPIOB)
#define NIRQ_PIN            (GPIO_PIN_8)
#define NRESET_GPIO_PORT    (GPIOA)
#define NRESET_PIN          (GPIO_PIN_2)

#define I2C_MODE                    (  0U)
#define SPI_MODE                    (  1U)

#define USB_DATABYTES_PER_WRITE (58U)

typedef enum
{
    eNoCommsSelected = 0,
    eI2C = 1,
    eSPI = 2,
} en_comms;

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

 
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
void        CommsInit(void);
void        CommsDeInit(void);
void        SetAxiomCommsMode(uint8_t comms_mode);
uint32_t    WriteAxiom(uint16_t addr, uint8_t *pbuf, uint32_t length);
uint32_t    ReadAxiom(uint16_t addr, uint8_t *pbuf, uint32_t length);
en_comms    GetCommsMode(void);
uint32_t    GetAxiomI2CAddress(void);
void        ResetAxiom(void);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* APPLICATION_INCLUDE_COMMS_H_ */
