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

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "flash_control.h"

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/


/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/


/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define FMC_KEY1    (0x45670123U)
#define FMC_KEY2    (0xCDEF89ABU)
#define FMC_OBKEY1  (0x45670123U)
#define FMC_OBKEY2  (0xCDEF89ABU)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void UnlockFlash(void);
static void LockFlash(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief   Erases option byte data 0 or data 1.
 * @param[in]   which_byte  The byte we wish to erase, whilst maintaining the other.
 *  @arg            ADDR_DATA0: Option byte data address 0
 *  @arg            ADDR_DATA1: Option byte data address 1
 * @return  None.
 */
void EraseOptionByte(uint32_t which_byte)
{
    UnlockFlash();

    // Wait for busy bit in FMC_STAT to clear (ensures no flash operations occurring)
    while (fmc_flag_get(FMC_FLAG_BUSY) == SET)
    {
        continue;
    }

    // Read full option byte data
    uint16_t ob_data = ob_data_get();

    // Wipe option byte
    ob_erase();

    // Write back option byte we didn't want to wipe
    switch (which_byte)
    {
        case ADDR_DATA0:
        {
            uint8_t ob_byte = (uint8_t)(ob_data >> 8U);
            ob_data_program(OB_DATA_ADDR1, ob_byte);
            break;
        }

        case ADDR_DATA1:
        {
            uint8_t ob_byte = (uint8_t)(ob_data & 0xFFU);
            ob_data_program(OB_DATA_ADDR0, ob_byte);
            break;
        }

        default:
        {
            // Do nothing.
        }
    }

    LockFlash();
}

/**
 * @brief   Writes a byte to the chosen option byte data address.
 * @param[in]   which_byte  The byte we wish to write to.
 *  @arg            ADDR_DATA0: Option byte data address 0
 *  @arg            ADDR_DATA1: Option byte data address 1
 * @param[in]   data    The byte to be written to the flash address.
 * @return  None.
 */
void WriteOptionByte(uint32_t which_byte, uint8_t data)
{
    UnlockFlash();

    // Wait for busy bit in FMC_STAT to clear (ensures no flash operations occurring)
    while (fmc_flag_get(FMC_FLAG_BUSY) == SET)
    {
        continue;
    }

    // Write to the chosen byte address
    switch (which_byte)
    {
        case ADDR_DATA0:
        {
            ob_data_program(OB_DATA_ADDR0, data);
            break;
        }

        case ADDR_DATA1:
        {
            ob_data_program(OB_DATA_ADDR1, data);
            break;
        }

        default:
        {
            // Do nothing.
            break;
        }
    }

    LockFlash();
}

/**
 * @brief   Must be performed after writing to the option byte otherwise changes will not be
 *          permanent.
 * @return  None.
 */
void CommitOptionByte(void)
{
    ob_reset();
}

/**
 * @brief   Returns the data byte at the chosen option byte data address.
 * @param[in]   which_byte  The byte we wish to read from.
 *  @arg            ADDR_DATA0: Option byte data address 0
 *  @arg            ADDR_DATA1: Option byte data address 1
 *  @return Value stored at the address.
 */
uint8_t ReadOptionByte(uint32_t which_byte)
{
    uint8_t data = 0;

    UnlockFlash();

    // Wait for busy bit in FMC_STAT to clear (ensures no flash operations occurring)
    while (fmc_flag_get(FMC_FLAG_BUSY) == SET)
    {
        continue;
    }

    uint16_t option_data = ob_data_get();

    switch (which_byte)
    {
        case ADDR_DATA0:
        {
            data = (uint8_t)(option_data & 0xFFU);
            break;
        }

        case ADDR_DATA1:
        {
            data = (uint8_t)(option_data >> 8U);
            break;
        }

        default:
        {
            // Do nothing.
            break;
        }
    }

    LockFlash();

    return data;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void UnlockFlash(void)
{
    fmc_unlock();
    ob_unlock();
}

static void LockFlash(void)
{
    fmc_lock();
    ob_lock();
}
