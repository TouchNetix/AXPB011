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
#include "device_control.h"
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
typedef  void (*app_func)(void);

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define FLASH_START_ADDR    (0x08000000U)   // Start of runtime code (Reset handler address)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void JumpToFlashStart(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
void RestartBridge(usb_core_driver *udev)
{
    DeviceDeInit(udev, DISABLE_IRQS);
    ob_reset(); // Can re-use this to create a reset
}

void EnterBootloader(usb_core_driver *udev)
{
    DeviceDeInit(udev, IRQ_UNCHANGED);

    // Write a magic word at the end of RAM - checked by the boot-loader
    *((unsigned long *)0x20001FF0) = 0xDEADBEEF;

    JumpToFlashStart();
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void JumpToFlashStart(void)
{
    // Set PC to reset vector.
    uint32_t app_addr = *(__IO uint32_t*) (FLASH_START_ADDR + 4U);
    app_func application = (app_func) app_addr;

    // Initialise user application's stack pointer.
    __set_MSP(*(__IO uint32_t*) FLASH_START_ADDR);

    // Jump to user application.
    application();
}
