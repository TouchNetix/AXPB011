/*!
 @file  i2c.h
 @brief I2C driver header file.

 @version 03/08/2022 - V01.00. First release. Author: James Cameron.
 */

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
#ifndef APPLICATION_INCLUDE_I2C_H_
#define APPLICATION_INCLUDE_I2C_H_

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
#define I2CSTATUS_READWRITEOK       (0x00U)
#define I2CSTATUS_COMMS_FAILED      (0x01U)
#define I2CSTATUS_TIMEOUT           (0x02U)
#define I2CSTATUS_WRITE_OK_NOREAD   (0x04U)

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

 
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
void        I2C_Init(void);
void        I2C_DeInit(void);
uint32_t    I2C_WriteAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length);
uint32_t    I2C_ReadAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length);
uint32_t    I2C_GetI2CAddress(void);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* APPLICATION_INCLUDE_I2C_H_ */
