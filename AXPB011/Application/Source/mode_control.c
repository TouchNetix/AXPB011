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
#include "mode_control.h"
#include "flash_control.h"
#include "init.h"

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
uint8_t g_bridge_mode = MODE_UNKNOWN;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define BRIDGE_MODE_ADDR    (ADDR_DATA0)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief   Checks for a valid bridge mode in the option byte. If not present,
 *          sets the bridge mode to digitizer.
 */
void CheckBridgeMode(usb_core_driver *udev)
{
    g_bridge_mode = ReadBridgeModeFromFlash();

    if ((g_bridge_mode == MODE_TBP_BASIC) || (g_bridge_mode == MODE_ABSOLUTE_MOUSE) || (g_bridge_mode == MODE_PARALLEL_DIGITIZER))
    {
        // Do nothing
    }
    else
    {
        WriteBridgeMode(udev, MODE_PARALLEL_DIGITIZER);
    }
}

/**
 * @brief   Writes the general bridge mode to the option byte.
 * @note    This will also cause the bridge to perform a system reset.
 * @param[in]   udev
 * @param[in]   bridge_mode
 * @return  None.
 */
void WriteBridgeMode(usb_core_driver *udev, uint8_t bridge_mode)
{
    EraseOptionByte(BRIDGE_MODE_ADDR);

    // Write new value
    WriteOptionByte(BRIDGE_MODE_ADDR, bridge_mode);

    // Needs a restart otherwise new option byte won't stick.
    // Must use FMC register to perform this.
    DeviceDeInit(udev, DISABLE_IRQS);
    CommitOptionByte();
}

/**
 * @brief Reads the general bridge mode from the option byte.
 * @return  Current bridge mode stored in option byte flash area.
 */
uint8_t ReadBridgeModeFromFlash(void)
{
    return ReadOptionByte(BRIDGE_MODE_ADDR);
}

/**
 * @brief Returns the general bridge mode.
 */
uint8_t GetBridgeMode(void)
{
    return g_bridge_mode;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/

