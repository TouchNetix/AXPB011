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
#include "dfu_core.h"
#include "drv_usb_hw.h"

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
bool bootloader_called_from_runtime = 0;

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/
#define RUN_APPLICATION (0U)

/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
usb_core_driver usb_dfu_dev;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define BOOT_GPIO_PORT  (GPIOA)
#define BOOT_GPIO_PIN   (GPIO_PIN_0)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void BootKeyInit(void);
static void BootKeyDeInit(void);
static void RunBootloader(void);
static void RunApplication(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
int main(void)
{
    // Test if user code is programmed starting from address 0x08004000. If not
    // then run the boot-loader (Taken from GD DFU example fw).
    if (0x20000000U == ((*(__IO uint32_t*) APP_LOADED_ADDR) & 0x2FFE0000U))
    {
        if (bootloader_called_from_runtime == FALSE)
        {
            BootKeyInit();

            // Check if boot select pin is asserted
            if (gpio_input_bit_get(BOOT_GPIO_PORT, BOOT_GPIO_PIN) == RUN_APPLICATION)
            {
                RunApplication();
            }
        }
    }

    // Not else - needs to run if any of the above checks fail
    RunBootloader();
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void BootKeyInit(void)
{
    // Enable clocks
    rcu_periph_clock_enable(RCU_GPIOA);

    // Configure button as input
    gpio_mode_set(BOOT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOOT_GPIO_PIN);
}

static void BootKeyDeInit(void)
{
    // Disable the clocks
    rcu_periph_clock_disable(RCU_GPIOA);

    // Clear the registers
    uint32_t ctl, pupd;

    ctl = GPIO_CTL(BOOT_GPIO_PORT);
    pupd = GPIO_PUD(BOOT_GPIO_PORT);

    for (uint16_t i = 0U; i < 16U; i++)
    {
        if ((1U << i) & BOOT_GPIO_PIN)
        {
            // Clear the specified pin mode bits
            ctl &= ~GPIO_MODE_MASK(i);

            // Clear the specified pin pupd bits
            pupd &= ~GPIO_PUPD_MASK(i);
        }
    }

    GPIO_CTL(BOOT_GPIO_PORT) = ctl;
    GPIO_PUD(BOOT_GPIO_PORT) = pupd;
}

static void RunBootloader(void)
{
    // Make sure all interrupts are enabled
    __enable_irq();

    // Reset GPIO registers
    BootKeyDeInit();

    usb_rcu_config();

    usb_timer_init();

    // USB device stack configure
    usbd_init(&usb_dfu_dev, USB_CORE_ENUM_FS, &dfu_desc, &dfu_class);

    usb_intr_config();

#ifdef USE_IRC48M
    // CTC peripheral clock enable
    rcu_periph_clock_enable(RCU_CTC);

    // CTC configure
    ctc_config();

    while(RESET == ctc_flag_get(CTC_FLAG_CKOK))
    {
        continue;
    }
#endif

    // check if USB device is enumerated successfully
    while (usb_dfu_dev.dev.cur_status != USBD_CONFIGURED)
    {
        continue;
    }

    for (;;)
    {
        continue;
    }
}

static void RunApplication(void)
{
    // Reset GPIO registers
    BootKeyDeInit();

    app_func application;
    uint32_t app_addr;

    // Set PC to reset vector
    app_addr = *(__IO uint32_t*) (APP_LOADED_ADDR + 4U);
    application = (app_func) app_addr;

    // Initialise user application's stack pointer
    __set_MSP(*(__IO uint32_t*) APP_LOADED_ADDR);

    // Jump to user application
    application();
}
