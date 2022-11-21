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
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "dfu_mal.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * Data Types
 ******************************************************************************/


/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define FLASH_START_ADDR        0x08000000
#define OB_RDPT                 0x1ffff800
#define MAL_MASK_OB             0xFFFFFF00
#define FLASH_END_ADDR          0x08300000
#define FLASH_IF_STRING         "@Internal Flash   /0x08000000/16*002Ka,112*002Kg"

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
extern dfu_mal_prop DFU_Flash_cb;
extern bool bootloader_called_from_runtime;

/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
fmc_state_enum Option_Byte_Write(uint32_t Mem_Add, uint8_t *data);

/*******************************************************************************
 * Macros
 ******************************************************************************/


/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* __FLASH_IF_H */
