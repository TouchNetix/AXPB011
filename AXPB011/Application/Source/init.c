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
#include "init.h"
#include "systick.h"
#include "drv_usb_hw.h"
#include "composite_usb_wrapper.h"
#include "digitizer_hid_core.h"
#include "comms.h"
#include "usage_controls.h"
#include "proxy_driver.h"
#include "timers_and_leds.h"
#include "mode_control.h"

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
extern generic_fop_handler      g_generic_fops;
extern press_fop_handler        g_press_fops;
extern digitizer_fop_handler    g_digitizer_fops;

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/


/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
typedef enum
{
    enAxiomConnected,
    enAxiomNotFound
} eAxiomConnectionStatus;

typedef enum
{
    enUsageTableParsed,
    enUsageTableEmpty
} enUsageTableState;

#define USB_HOST_DETECTED_COUNT_LIMIT   (2000U)

#define USAGETABLE_MAX_RETRY_NUM        (  250U)
#define MAX_USB_WAIT                    (20000U)

#define MAX_NUM_HID_PARAMETERS          ( 16U)
#define VID                             (  1U)
#define PID                             (  2U)
#define PHYS_X                          (  3U)
#define PHYS_Y                          (  4U)
#define LOGMAX_X                        (  5U)
#define LOGMAX_Y                        (  6U)
#define WAKEUP_OPTION                   (  7U)

#define USB_GPIO_PORT                   (GPIOA)
#define USB_DM_PIN                      (GPIO_PIN_11)
#define USB_DP_PIN                      (GPIO_PIN_12)

// Constants used for array manipulation
#define PHYS_X_FIRST_TOUCH_BYTELO   (58)
#define PHYS_X_FIRST_TOUCH_BYTE2    (59)
#define PHYS_X_FIRST_TOUCH_BYTE3    (60)
#define PHYS_X_FIRST_TOUCH_BYTEHI   (61)
#define PHYS_Y_FIRST_TOUCH_BYTELO   (72)
#define PHYS_Y_FIRST_TOUCH_BYTE2    (73)
#define PHYS_Y_FIRST_TOUCH_BYTE3    (74)
#define PHYS_Y_FIRST_TOUCH_BYTEHI   (75)
#define LOGMAX_X_FIRST_TOUCH_LO     (53)
#define LOGMAX_X_FIRST_TOUCH_HI     (54)
#define LOGMAX_Y_FIRST_TOUCH_LO     (67)
#define LOGMAX_Y_FIRST_TOUCH_HI     (68)
#define ARRAY_CONST                 (78)    // number of bytes between touches in digitizer report descriptor

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/
#define HID_PARAMETER_ID(n)     (n * 3) // n is the field ID index (maps to those found in aXiom config)
#define LOBYTE(x)  ((uint8_t)(x & 0x00FF))
#define HIBYTE(x)  ((uint8_t)((x & 0xFF00) >>8))

/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static uint8_t CheckUSBHostPresence(void);
static enUsageTableState ParseUsageTable(void);
static void GPIOConfig(void);
static enUsageTableState ReadAndBuildUsageTable(void);
static eAxiomConnectionStatus CheckAxiomPresence(uint8_t *pBuf);
static void USB_FS_Init(usb_core_driver *pDevice);
static void USB_FS_DeInit(usb_core_driver *pDevice);
static void ConfigureDescriptors(void);
static void ConfigureHIDParametersFromU35(void);
static void GPIO_DeInit(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
uint8_t DeviceInit(usb_core_driver *pDevice, uint8_t SkipInitialSetup)
{
    uint8_t status = USB_HOST_ABSENT;

    // These bits must only be called once, subsequent calls to this function should skip the initial setup.
    if (SkipInitialSetup == RUN_INITIAL_SETUP)
    {
        // If we've done a software reset, we need to re-enable interrupts
        __enable_irq();

        // Configure systick
        systick_config();

        CheckBridgeMode(pDevice);

        // Sets up nRESET, nIRQ - we need nRESET to change the mode of axiom
        GPIOConfig();
        LEDsInit();
    }

    if ( CheckUSBHostPresence() == USB_HOST_ABSENT )
    {
        // Don't release the nSLVI2C/MISO line - if the new host resets axiom it should remain in I2C mode
        SetAxiomCommsMode( I2C_MODE );
        status = USB_HOST_ABSENT;
    }
    else
    {
        status = USB_HOST_DETECTED;
    }

    // Only continue to configure the bridge if the USB host was detected.
    if (status == USB_HOST_DETECTED)
    {
        // Configure the rest of the peripherals and reset axiom to put it in the correct comms mode (during comms init)
        TimerInit();
        CommsInit();

        enUsageTableState usagetable_status = enUsageTableEmpty;

        // If we're in I2C mode and aXiom hasn't been found, don't bother trying to read the usage table.
        // We've waited ~12s already, so just come up in Bridge Only mode (won't be able to read usage table anyway).
        if ( ( GetCommsMode() == eSPI ) ||
                ( ( GetCommsMode() == eI2C ) && ( GetAxiomI2CAddress() != 0) ) )
        {
            usagetable_status = ParseUsageTable();
        }

        // Return both LEDs to the same state
        gpio_bit_write(LED_AXIOM_GPIO_PORT, LED_AXIOM_GPIO_PIN, RESET);
        gpio_bit_write(LED_USB_GPIO_PORT, LED_USB_GPIO_PIN, RESET);

        delay_1ms(1000);

        // Enable USB - happens last when everything else is set up (at this point we are ready to connect to the host)
        USB_FS_Init( pDevice );

        if ( usagetable_status == enUsageTableEmpty )
        {
            // Usage table couldn't be parsed, meaning axiom couldn't be connected to - no reports to read!
            SetProxyMode( eProxyStop );
        }
        else
        {
            // Start the timer - enables ePressDigiProxy after ~1 second delay
            StartProxyDelayTimer();
        }
    }

    return status;
}

void DeviceDeInit(usb_core_driver *pDevice, uint8_t disable_irq)
{
    USB_FS_DeInit(pDevice);
    delay_1ms(2000);
    CommsDeInit();
    GPIO_DeInit();

    if (disable_irq == DISABLE_IRQS)
    {
        __disable_irq();
    }
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static uint8_t CheckUSBHostPresence(void)
{
    uint8_t status = USB_HOST_ABSENT;

    // Initialise USB GPIOs as inputs
    gpio_mode_set(USB_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, USB_DM_PIN);
    gpio_mode_set(USB_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, USB_DP_PIN);

    uint16_t host_detection_time_limit = 0;
    uint16_t host_detected_count = 0;
    do
    {
        /* Physical connections of the USB data line (before USB module is enabled)
         * Host side:
         *      ~20k pull downs on D+ and D-
         * Bridge side:
         *      220k pull ups on D+ and D- (weak enough to not interfere with USB transmissions)
         *
         * If host is present:
         *      D+ and D- will read 0
         *
         * Host not present
         *      D+ and D- will read 1
         *
         * In the event of a transient (such as emc testing) we need to be certain the host is actually there,
         * so we check for multiple loops and reset our counter if we 'lose' the host
         */
        if ((gpio_input_bit_get(USB_GPIO_PORT, USB_DM_PIN) == RESET) && (gpio_input_bit_get(USB_GPIO_PORT, USB_DP_PIN) == RESET))
        {
            if(host_detected_count < USB_HOST_DETECTED_COUNT_LIMIT)
            {
                host_detected_count++;
            }
        }
        else
        {
            host_detected_count = 0;
        }

        if(host_detected_count >= USB_HOST_DETECTED_COUNT_LIMIT)
        {
            status = USB_HOST_DETECTED;
        }
        else
        {
            status = USB_HOST_ABSENT;
        }

        host_detection_time_limit++;
    } while(host_detection_time_limit <= MAX_USB_WAIT);

    return status;
}

static enUsageTableState ParseUsageTable(void)
{
    // The following do-while loop will run for ~12.5 seconds if no axiom can be found.
    // (Enough time for axiom to exit bootloader)
    uint8_t retry = 0;
    enUsageTableState usagetable_status = enUsageTableEmpty;

    // Starting this timer starts to alternate the LEDs (searching for aXiom mode)
    timer_enable(STARTUP_LED_TIMER);

    do
    {
        usagetable_status = ReadAndBuildUsageTable();

        if (usagetable_status == enUsageTableParsed)
        {
            break;
        }
        else
        {
            // Wait for a bit before trying again
            delay_1ms(50);
        }

        retry++;
    } while(retry < USAGETABLE_MAX_RETRY_NUM);

    // Clean up in this function - better than relying on an external function having to disable this for us
    timer_disable(STARTUP_LED_TIMER);

    return usagetable_status;
}

static void GPIOConfig(void)
{
    // Enable clocks (SPI/I2C enabled in their respective functions)
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_DMA);

    gpio_mode_set(NIRQ_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, NIRQ_PIN);

    // Make sure nRESET is initialised high
    gpio_bit_write(NRESET_GPIO_PORT, NRESET_PIN, SET);
    gpio_mode_set(NRESET_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, NRESET_PIN);
    gpio_output_options_set(NRESET_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, NRESET_PIN);
}

static enUsageTableState ReadAndBuildUsageTable(void)
{
    // Read the number of usages known to axiom
    uint8_t temp_buffer[DEVICE_INFO_LENGTH] = {0};
    uint8_t num_usages = 0;

    // Check nIRQ state, if aXiom is ready it will pull it low (indicating report to read)
    if (gpio_input_bit_get(NIRQ_GPIO_PORT, NIRQ_PIN) == RESET)
    {
        ReadAxiom(DEVICE_INFO_ADDRESS, temp_buffer, DEVICE_INFO_LENGTH);
        num_usages = temp_buffer[NUM_USAGES_OFFSET];

        // Check is axiom is connected and able to communicate
        if (CheckAxiomPresence(temp_buffer) == enAxiomNotFound)
        {
            return enUsageTableEmpty;
        }

        // Prevents a memory leak, although requires updating this firmware if this ever changes
        if (num_usages > MAX_NUM_USAGES)
        {
            num_usages = MAX_NUM_USAGES;
        }

        // Parse the usage table from axiom and store a local copy
        ClearLocalUsageTable();

        uint8_t  usagetable_buffer[6U] = {0};
        uint16_t next_addr = USAGE_TABLE_ADDRESS;
        uint8_t  usage_table_idx = 0;

        while (usage_table_idx < num_usages)
        {
            ReadAxiom(next_addr, usagetable_buffer, USAGE_ENTRY_SIZE);
            SaveUsageInLocalTable(usage_table_idx, usagetable_buffer, USAGE_ENTRY_SIZE);

            next_addr += USAGE_ENTRY_SIZE;
            usage_table_idx++;
        }

        return enUsageTableParsed;
    }
    else
    {
        // aXiom not ready for comms yet
        return enUsageTableEmpty;
    }
}

static eAxiomConnectionStatus CheckAxiomPresence(uint8_t *pBuf)
{
    eAxiomConnectionStatus status = enAxiomNotFound;

    // If axiom is connected but 'dead' or disconnected the bridge will read back all 0xFF (or 0x00).
    // If all 12 bytes read back as 0xFF or 0x00 then return an error.
    for(uint8_t i = 0; i < 12; i++)
    {
        if(pBuf[i] != 0xFF && pBuf[i] != 0x00)
        {
            // If a byte is NOT 0xFF we assume aXiom is present and alive
            status = enAxiomConnected;
        }
    }

    return status;
}

static void USB_FS_Init(usb_core_driver *pDevice)
{
    // Alter descriptors depending on mode
    ConfigureDescriptors();

    // Check for user VID and PID - u35 overwrites any internal bridge values
    ConfigureHIDParametersFromU35();

    // Configure USB clock
    usb_rcu_config();

    // Timer initialization
    usb_timer_init();

    // Configure key and led
    generic_hid_itfop_register(pDevice, &g_generic_fops);
    press_hid_itfop_register(pDevice, &g_press_fops);
    digitizer_hid_itfop_register(pDevice, &g_digitizer_fops);

    // USB device stack configure
    usbd_init(pDevice, USB_CORE_ENUM_FS, &hid_composite_desc, &usbd_hid_composite_cb);

    // USB interrupt configure
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
}

static void USB_FS_DeInit(usb_core_driver *pDevice)
{
    // Disconnect from the host. Allow some time for the host to remove us from their device list.
    usbd_disconnect(pDevice);
    delay_1ms(1000);
    usb_core_reset(&pDevice->regs);

    usbd_generic_hid_cb.deinit(pDevice, 0);
    usbd_press_hid_cb.deinit(pDevice, 0);
    usbd_digitizer_hid_cb.deinit(pDevice, 0);
    usb_core_reset(&pDevice->regs);
    rcu_periph_clock_disable(RCU_USBFS);

    // Make sure everything is reset
    rcu_periph_reset_enable(RCU_USBFSRST);
    rcu_periph_reset_disable(RCU_USBFSRST);
}

static void ConfigureDescriptors(void)
{
    uint8_t bridge_mode = GetBridgeMode();

    switch (bridge_mode)
    {
        case MODE_TBP_BASIC:
        {
            composite_dev_desc.idProduct = DEVICE_MODE_PID_TBP;

            // Set number of interfaces
            composite_config_desc.config.bNumInterfaces = 2;

            // Set Cfg descriptor length
            composite_config_desc.config.wTotalLength = USB_COMPOSITE_CONFIG_DESC_LEN_NO_DIGITIZER;

            // Set digitizer report descriptor length
            composite_config_desc.digitizer_hid_descriptor.wDescriptorLength = 0;

            // Set interface string descriptor index
            composite_config_desc.digitizer_interface.iInterface = STR_IDX_ITF_DIGITIZER;

            // Set digitizer report length
            composite_config_desc.digitizer_epin.wMaxPacketSize = 0;

            break;
        }

        case MODE_PARALLEL_DIGITIZER:
        {
            composite_dev_desc.idProduct = DEVICE_MODE_PID_PARALLEL_DIGITIZER;

            // Set number of interfaces
            composite_config_desc.config.bNumInterfaces = 3;

            // Set Cfg descriptor length
            composite_config_desc.config.wTotalLength = USB_COMPOSITE_CONFIG_DESC_LEN;

            // Set digitizer report descriptor length
            composite_config_desc.digitizer_hid_descriptor.wDescriptorLength = DIGITIZER_REPORT_DESC_LEN;

            // Set interface string descriptor index
            composite_config_desc.digitizer_interface.iInterface = STR_IDX_ITF_DIGITIZER;

            // Set digitizer report length
            composite_config_desc.digitizer_epin.wMaxPacketSize = DIGITIZER_IN_PACKET_SIZE;

            break;
        }

        case MODE_ABSOLUTE_MOUSE:
        {
            composite_dev_desc.idProduct = DEVICE_MODE_PID_ABSOLUTE_MOUSE;

            // Set number of interfaces
            composite_config_desc.config.bNumInterfaces = 3;

            // Set Cfg descriptor length
            composite_config_desc.config.wTotalLength = USB_COMPOSITE_CONFIG_DESC_LEN;

            // Set digitizer report descriptor length
            composite_config_desc.digitizer_hid_descriptor.wDescriptorLength = ABS_MOUSE_REPORT_DESC_LEN;

            // Set interface string descriptor index
            composite_config_desc.digitizer_interface.iInterface = STR_IDX_ITF_MOUSE;

            // Set digitizer report length
            composite_config_desc.digitizer_epin.wMaxPacketSize = MOUSE_ABS_IN_PACKET_SIZE;

            break;
        }

        default:
        {
            // Do nothing.
            break;
        }
    }
}

static void ConfigureHIDParametersFromU35(void)
{
    int8_t usage_index = FindUsageInTable(0x35);

    // usage_index will return -1 if usage couldn't be found
    if (usage_index > -1)
    {
        usagetableentry_st *usage_info = GetUsageInfo(usage_index);

        uint8_t u35_buffer[MAX_NUM_HID_PARAMETERS * 3] = {0};
        ReadAxiom((uint16_t)(usage_info->startpage << 8U), u35_buffer, MAX_NUM_HID_PARAMETERS * 3);

        // Loop through u35 and check for any user defined parameters
        uint8_t parameter_count = 0;
        while (parameter_count < MAX_NUM_HID_PARAMETERS)
        {
            // First byte contains the ID of the parameter
            switch(u35_buffer[HID_PARAMETER_ID(parameter_count)])
            {
                case VID:
                {
                    composite_dev_desc.idVendor = (uint16_t)((u35_buffer[2 + HID_PARAMETER_ID(parameter_count)] << 8U) | (u35_buffer[1 + HID_PARAMETER_ID(parameter_count)] & 0xFFU));
                    break;
                }

                case PID:
                {
                    composite_dev_desc.idProduct = (uint16_t)((u35_buffer[2 + HID_PARAMETER_ID(parameter_count)] << 8U) | (u35_buffer[1 + HID_PARAMETER_ID(parameter_count)] & 0xFFU));
                    break;
                }

                case PHYS_X:
                {
                    uint16_t PhysMaxX_Temp = (uint16_t)((u35_buffer[2 + HID_PARAMETER_ID(parameter_count)] << 8U) | (u35_buffer[1 + HID_PARAMETER_ID(parameter_count)] & 0xFFU));
                    uint32_t PhysMaxX = PhysMaxX_Temp * 5;   // multiply the read value by 5 to produce the value requested by the user --> TH2 assumes this value increases in steps of 0.5mm, but spec states an increase step of 0.1mm

                    for (uint8_t touch_num = 0; touch_num < MAX_NUM_DIGITIZER_TOUCHES; touch_num++)
                    {
                        digitizer_report_descriptor[PHYS_X_FIRST_TOUCH_BYTELO + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxX & 0x000000FF) >> 0);  // Low byte
                        digitizer_report_descriptor[PHYS_X_FIRST_TOUCH_BYTE2  + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxX & 0x0000FF00) >> 8);  // Second byte
                        digitizer_report_descriptor[PHYS_X_FIRST_TOUCH_BYTE3  + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxX & 0x00FF0000) >> 16); // Third byte
                        digitizer_report_descriptor[PHYS_X_FIRST_TOUCH_BYTEHI + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxX & 0xFF000000) >> 24); // High byte
                    }

                    break;
                }

                case PHYS_Y:
                {
                    uint16_t PhysMaxY_Temp = (uint16_t)((u35_buffer[2 + HID_PARAMETER_ID(parameter_count)] << 8U) | (u35_buffer[1 + HID_PARAMETER_ID(parameter_count)] & 0xFFU));
                    uint32_t PhysMaxY = PhysMaxY_Temp * 5;   // multiply the read value by 5 to produce the value requested by the user --> TH2 assumes this value increases in steps of 0.5mm, but spec states an increase step of 0.1mm

                    for (uint8_t touch_num = 0; touch_num < MAX_NUM_DIGITIZER_TOUCHES; touch_num++)
                    {
                        digitizer_report_descriptor[PHYS_Y_FIRST_TOUCH_BYTELO + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxY & 0x000000FF) >> 0);  // Low byte
                        digitizer_report_descriptor[PHYS_Y_FIRST_TOUCH_BYTE2  + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxY & 0x0000FF00) >> 8);  // Second byte
                        digitizer_report_descriptor[PHYS_Y_FIRST_TOUCH_BYTE3  + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxY & 0x00FF0000) >> 16); // Third byte
                        digitizer_report_descriptor[PHYS_Y_FIRST_TOUCH_BYTEHI + (touch_num * ARRAY_CONST)] = (uint8_t)((PhysMaxY & 0xFF000000) >> 24); // High byte
                    }

                    break;
                }

                case LOGMAX_X:
                {
                    uint16_t LogMaxX = (uint16_t)((u35_buffer[2 + HID_PARAMETER_ID(parameter_count)] << 8U) | (u35_buffer[1 + HID_PARAMETER_ID(parameter_count)] & 0xFFU));

                    for (uint8_t touch_num = 0; touch_num < MAX_NUM_DIGITIZER_TOUCHES; touch_num++)
                    {
                        digitizer_report_descriptor[LOGMAX_X_FIRST_TOUCH_LO + (touch_num * ARRAY_CONST)] = LOBYTE(LogMaxX); // Low byte
                        digitizer_report_descriptor[LOGMAX_X_FIRST_TOUCH_HI + (touch_num * ARRAY_CONST)] = HIBYTE(LogMaxX); // High byte
                    }

                    break;
                }

                case LOGMAX_Y:
                {
                    uint16_t LogMaxY = (uint16_t)((u35_buffer[2 + HID_PARAMETER_ID(parameter_count)] << 8U) | (u35_buffer[1 + HID_PARAMETER_ID(parameter_count)] & 0xFFU));

                    for (uint8_t touch_num = 0; touch_num < MAX_NUM_DIGITIZER_TOUCHES; touch_num++)
                    {
                        digitizer_report_descriptor[LOGMAX_Y_FIRST_TOUCH_LO + (touch_num * ARRAY_CONST)] = LOBYTE(LogMaxY); // Low byte
                        digitizer_report_descriptor[LOGMAX_Y_FIRST_TOUCH_HI + (touch_num * ARRAY_CONST)] = HIBYTE(LogMaxY); // High byte
                    }

                    break;
                }

                case WAKEUP_OPTION:
                {
                    SetHostWakeOption(u35_buffer[1 + HID_PARAMETER_ID(parameter_count)]);
                    break;
                }

                default:
                {
                    break;
                }
            }

            parameter_count++;
        }
    }
}

static void GPIO_DeInit(void)
{
    // Make sure all GPIOs are reset. Disable clocks.
    gpio_deinit(GPIOA);
    gpio_deinit(GPIOB);
    gpio_deinit(GPIOC);
    rcu_periph_clock_disable(RCU_GPIOA);
    rcu_periph_clock_disable(RCU_GPIOB);
    rcu_periph_clock_disable(RCU_GPIOC);
}
