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
#include "command_processor.h"
#include "comms.h"
#include "usage_controls.h"
#include "proxy_driver.h"
#include "device_control.h"
#include "systick.h"
#include "press_hid_itf.h"
#include "digitizer_hid_itf.h"
#include "mode_control.h"
#include "timers_and_leds.h"

#include <string.h>

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
#define READ                            (0x80)
#define WRITE                           (0x00)
#define PROXY_SETTINGS_OK               (0x00u)
#define INVALID_SETTINGS                (0x01u)
#define INVALID_COMMAND                 (0x99u)
#define MODE_SWITCH_OK                  (0xE7u)
#define SPI_MODE_ADDRESS                (0x01u)
#define I2C_ERROR                       (0x81u)
#define UNKNOWN_DEVICE                  (0xFFu)
#define ID_F042                         (0x0Au)
#define ID_F070                         (0x0Bu)
#define ID_F072                         (0x0Cu)
#define ID_GDF350                       (0x0Du)
#define NUM_CMD_BYTES                   (4U)
#define MAX_NUMBYTES_PER_AXIOM_READ     (62U)
#define MAX_NUMBYTES_PER_AXIOM_WRITE    (57U)
#define MAX_NUMBYTES_PER_CMDWRITEREAD   (59U)   /* Maximum number of bytes that can be read or written to a usage by the CMD_READ_USAGE or CMD_WRITE_USAGE commands */

// Error codes for the read/write usage command
#define EXCESS_BYTES                    (92U)   /* Too many bytes requested                       */
#define INVALID_OFFSET                  (93U)   /* Offset into page addresses beyond end of USAGE */
#define INVALID_STARTPAGE               (94U)   /* Start page too big                             */
#define INVALID_USAGENUM                (95U)   /* USAGE is a report placeholder and not writable */
#define UNKNOWN_USAGENUM                (96U)   /* USAGE unknown to bridge                        */
#define COMMS_ERROR                     (97U)   /* Comms error (internal)                         */
#define INCORRECT_BRIDGE_MODE           (98U)   /* Bridge not in correct mode to run function     */
#define READWRITE_ERROR                 (80U)   /* Error occurred                                 */
#define PASS                            (1U)    /* Return status for read/write usage checks      */
#define FAIL                            (0U)    /* Return status for read/write usage checks      */

//------------TBP Commands------------
#define CMD_ZERO                        (0x00u)     /* TH2 will call this as it exits - stops proxy mode, starts counter to re-enable proxy mode once it has closed.                                         */
#define CMD_AXIOM_COMMS                 (0x51u)     /* Used by TH2 to read/write from the bridge.                                                                                                            */
#define CMD_MULTIPAGE_READ              (0x71u)     /* Automatically performs multiple reads and USB transfers from a usage.                                                                                 */
#define CMD_NULL                        (0x86u)     /* Cancels proxy if sent to control endpoint.                                                                                                            */
#define CMD_START_PROXY                 (0x88u)     /* Bridge continually reads reports from aXiom and sends to host via control endpoint. Also starts reports on the press and digi endpoints (if enabled). */
#define CMD_RESET_AXIOM                 (0x99u)     /* Sends a reset signal to axiom.                                                                                                                        */
#define CMD_WRITE_USAGE                 (0xA2u)     /* Allows host to write to a usage without needing to know the exact register address.                                                                   */
#define CMD_READ_USAGE                  (0xA3u)     /* Allows host to read a usage without needing to know the exact register address.                                                                       */
#define CMD_FIND_I2C_ADDRESS            (0xE0u)     /* Returns the I2C address of aXiom, or reports as in SPI mode.                                                                                          */

//------------Mode Control Commands------------
#define CMD_RESET_BRIDGE                (0xEFu)     /* Bridge performs a self-reset.                                        */
#define CMD_GET_PART_ID                 (0xF0u)     /* Returns an id specific to the bridge.                                */
#define CMD_ENTER_BOOTLOADER            (0xF5u)     /* Bridge enters DFU bootloader mode.                                   */
#define CMD_GET_BRIDGE_MODE             (0xF9u)     /* Returns the current bridge mode: Basic, Digitizer or Absolute Mouse. */
#define CMD_SWITCH_MODE_TBP_BASIC       (0xFAu)     /* Bridge enters basic mode.                                            */
#define CMD_SWITCH_MODE_TBP_DIGITIZER   (0xFEu)     /* Bridge enters digitizer mode.                                        */
#define CMD_SWITCH_MODE_TBP_ABS_MOUSE   (0xFFu)     /* Bridge enters absolute mouse mode.                                   */
#define CMD_BLOCK_DIGITIZER_REPORTS     (0x87u)     /* Enables/disables mouse reports.                                      */
#define CMD_BLOCK_PRESS_REPORTS         (0xB1u)     /* Enables/disables press reports.                                      */

//------------TH2 Compatibility Commands------------
#define CMD_SET_CONFIG                  (0x80u)     /* Sets parameters for bridge to use. */
#define CMD_GET_CONFIG                  (0x8Bu)     /* TH2 reads some operating parameters from the bridge. */

//------------Reserved Commands------------
#define CMD_IIC_DATA_2                      (0x52u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_3                      (0x53u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_4                      (0x54u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_5                      (0x55u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_256_EXTRA              (0x60u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_1_256                  (0x61u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_2_256                  (0x62u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_3_256                  (0x63u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_4_256                  (0x64u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_5_256                  (0x65u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_x256                   (0x68u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_1_x256                 (0x69u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_2_x256                 (0x6Au) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_3_x256                 (0x6Bu) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_4_x256                 (0x6Cu) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_5_x256                 (0x6Du) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_2_MP                   (0x72u) /* RESERVED - Used by the PB005/7 */
#define CMD_CONFIG_READ_PINS                (0x81u) /* RESERVED - Used by the PB005/7 */
#define CMD_GET_VOLTAGE                     (0xA0u) /* RESERVED - Used by the PB005/7 */
#define CMD_CALL_SELFTEST                   (0xA1u) /* RESERVED - Used by the PB005/7 */
#define CMD_CLOCK_PRESS                     (0xB0u) /* RESERVED - Used by the PB005/7 */
#define CMD_RnR_PEAK_PRESS                  (0xC0u) /* RESERVED - Used by the PB005/7 */
#define CMD_FLIP_MOUSE_AXES                 (0xD0u) /* RESERVED - Used by the PB005/7 */
#define CMD_SET_IO_A                        (0xE4u) /* RESERVED - Used by the PB005/7 */
#define CMD_GET_IO_A                        (0xE5u) /* RESERVED - Used by the PB005/7 */
#define CMD_SET_IO_B                        (0xE6u) /* RESERVED - Used by the PB005/7 */
#define CMD_GET_IO_B                        (0xE7u) /* RESERVED - Used by the PB005/7 */
#define CMD_SET_IO_C                        (0xE8u) /* RESERVED - Used by the PB005/7 */
#define CMD_GET_IO_C                        (0xE9u) /* RESERVED - Used by the PB005/7 */
#define CMD_SAVE_CONFIGS_EEPROM             (0xEAu) /* RESERVED - Used by the PB005/7 */
#define CMD_RESTORE_DEFAULT_CONFIGS         (0xEBu) /* RESERVED - Used by the PB005/7 */
#define CMD_SWITCH_USB                      (0xF6u) /* RESERVED - Used by the PB005/7 */
#define CMD_CHECK_UUT_USB_ACTIVITY          (0xF7u) /* RESERVED - Used by the PB005/7 */
#define CMD_IIC_DATA_MASK                   (0xF8u) /* RESERVED - Used by the PB005/7 */
#define CMD_SWITCH_MODE_DEBUG               (0xFCu) /* RESERVED - Used by the PB005/7 */
#define CMD_SWITCH_MODE_SERIAL_DIGITIZER    (0xFDu) /* RESERVED - Used by the PB005/7 */

/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/


/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static uint8_t ReadWriteUsageErrorChecks(uint8_t *pCommand, uint8_t *pResponse, usagetableentry_st *usage_info);


/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief
 * @param[in]   udev            Pointer to USB device instance.
 * @param[out]  pbuf            Pointer to buffer where response will be written to.
 * @param[in]   which_interface Command was sent to control or press interface.
 * @return  Whether an immediate response is required.
 */
uint8_t ProcessTBPCommand(usb_core_driver *udev, uint8_t *pResponse, uint8_t which_interface)
{
    uint8_t commandBuf[USBFS_MAX_PACKET_SIZE] = {0};
    enBridgeProxyMode proxy_mode_cache = eProxyStop;

    if (which_interface == CONTROL_INTERFACE)
    {
        generic_hid_handler *hhid = (generic_hid_handler *)udev->dev.class_data[GENERIC_INTERFACE_NUM];
        memcpy(&commandBuf[0], (uint8_t *)&hhid->data[0], GENERIC_HID_OUT_PACKET_SIZE);
        proxy_mode_cache = GetCurrentProxyMode();
        SetProxyMode(eProxyStop);
    }
    else if (which_interface == PRESS_INTERFACE)
    {
        press_hid_handler *hhid = (press_hid_handler *)udev->dev.class_data[PRESS_INTERFACE_NUM];
        memcpy(&commandBuf[0], (uint8_t *)&hhid->data[0], PRESS_HID_OUT_PACKET_SIZE);
        proxy_mode_cache = GetCurrentProxyMode();
    }
    else
    {
        // Incorrect interface
        return NO_RESPONSE_REQUIRED;
    }

    uint8_t response_required = NO_RESPONSE_REQUIRED;

    switch (commandBuf[0])
    {

//------------------------------------------------------------TBP COMMANDS------------------------------------------------------------

        case CMD_ZERO:
        {
            // Called by TH2 during exit
            // Start the timer - enables ePressDigiProxy after ~1 second delay
            StartProxyDelayTimer();

            pResponse[0] = CMD_ZERO;
            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_AXIOM_COMMS:
        {
            uint16_t address = (uint16_t)(commandBuf[4] << 8U) | (uint16_t)(commandBuf[3] & 0x00FFU);
            uint8_t numBytesRead = commandBuf[2];

            if (commandBuf[6] & READ)
            {
                if (numBytesRead > MAX_NUMBYTES_PER_AXIOM_READ)
                {
                    pResponse[0] = AXIOMCOMMS_INCORRECTSETUP;
                    pResponse[1] = 0;
                }
                else
                {
                    ReadAxiom(address, &pResponse[2], numBytesRead);
                    pResponse[0] = AXIOMCOMMS_OK;
                    pResponse[1] = numBytesRead;
                }
            }
            else
            {
                if (numBytesRead > MAX_NUMBYTES_PER_AXIOM_WRITE)
                {
                    pResponse[0] = AXIOMCOMMS_INCORRECTSETUP;
                    pResponse[1] = 0;
                }
                else
                {
                    // We already account for the 4 command bytes, so subtract them from the total number to write
                    WriteAxiom(address, &commandBuf[7], (commandBuf[1] - NUM_CMD_BYTES));
                    pResponse[0] = AXIOMCOMMS_OK_NO_READ;
                    pResponse[1] = 0; // No data read during a write
                }
            }

            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_MULTIPAGE_READ:
        {
            MultiPageSetupTypeDef setup;
            setup.SemaphoreUsageNumber  = commandBuf[7];
            setup.SemaphoreOffset       = commandBuf[8];
            setup.SemaphoreStartByte    = commandBuf[9];
            setup.SemaphoreStopByte     = commandBuf[10];
            setup.NumTx                 = (commandBuf[1] - NUM_CMD_BYTES); // We already account for the 4 command bytes, so subtract them from the total number to write
            setup.TotalNumRX            = (uint16_t)((commandBuf[3] << 8) | (uint16_t)commandBuf[2]);
            setup.PageLength            = commandBuf[4];
            setup.AddrStart             = (uint16_t)((commandBuf[6] << 8) | commandBuf[5]);
            setup.BytesRead             = 0;
            setup.PagesMovedThrough     = 0;
            setup.BytesOffset           = 0;

            // Store local copies of the start and stop byte sent in the command
            StoreMultipageReadInfo(&setup);

            // Send the start semaphore
            WriteSemaphore(SEMAPHORE_START);

            // Change the proxy mode - rest of reads will be done in main loop
            SetProxyMode(eMultiPageRead);

            response_required = NO_RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_NULL:
        {
            pResponse[0] = CMD_NULL;
            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_START_PROXY:
        {
            // Check u34 address isn't empty
            if ((commandBuf[2] == NUM_CMD_BYTES) && ((commandBuf[7] & 0x80) == READ))
            {
                Updateu34Addr((uint16_t)(commandBuf[5] << 8) | commandBuf[4]);

                // Requested by host, so start proxy mode on all enabled interfaces
                SetProxyMode(eControlPressDigiProxy);
                pResponse[1] = PROXY_SETTINGS_OK;
            }
            else
            {
                SetProxyMode(eProxyStop);
                pResponse[1] = INVALID_SETTINGS;
            }

            pResponse[0] = CMD_START_PROXY;
            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_RESET_AXIOM:
        {
            ResetAxiom();

            pResponse[0] = CMD_RESET_AXIOM;
            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_WRITE_USAGE:
        {
            /* WRITE USAGE
             * Command bytes
             * 1: usage number (in hex)
             * 2: relative start page (0 means 1st page, 1 means 2nd page etc.)
             * 3: byte offset into page
             * 4: no. bytes to write (min 0, max usage_len_bytes - 1)
             * 5+: data to write
             *
             * RETURN
             * 1-64: echo command
             */

            uint8_t usage_number    = commandBuf[1];
            uint8_t start_page      = commandBuf[2];
            uint8_t byte_offset     = commandBuf[3];
            uint8_t num_bytes       = commandBuf[4];

            int8_t usage_table_idx = FindUsageInTable(usage_number);

            if (usage_table_idx < 0)
            {
                pResponse[1] = UNKNOWN_USAGENUM;
                pResponse[2] = READWRITE_ERROR;
            }
            else
            {
                memcpy(&pResponse[0], &commandBuf[0], GENERIC_IN_PACKET_SIZE); // Put command in as echo response

                // Calculate how long the usage in in bytes
                usagetableentry_st *usage_info = GetUsageInfo(usage_table_idx);

                // Check for errors in request
                uint8_t check_for_errors = ReadWriteUsageErrorChecks(commandBuf, pResponse, usage_info);

                // If FAIL don't do anything, error codes have been setup in ReadWriteUsageErrorChecks()
                if (check_for_errors == PASS)
                {
                    uint16_t start_address = ((usage_info->maxoffset & 0x80) == 0x00) ?
                                        ((uint16_t)usage_info->startpage << 8) + (((uint16_t)start_page * ((uint16_t)(usage_info->maxoffset & 0x7F) + 1)) * 2) + byte_offset :
                                        ((uint16_t)usage_info->startpage << 8) + (((uint16_t)start_page * 2) + byte_offset);

                    // Although it would be smart for the bridge to send multiple USB response packets if more than an endpoints worth has been requested, previous bridges
                    // have not done this. In order to maintain backwards compatibility with hosts/software it will be assumed the host is smart enough to split up the reads itself.
                    if (num_bytes > MAX_NUMBYTES_PER_CMDWRITEREAD)
                    {
                        num_bytes = MAX_NUMBYTES_PER_CMDWRITEREAD;
                    }

                    WriteAxiom(start_address, &commandBuf[5], num_bytes);
                }
            }

            response_required = RESPONSE_REQUIRED;
            break;
        }

        case CMD_READ_USAGE:
        {
            /*
             * READ USAGE
             * 1: usage number
             * 2: start page
             * 3: byte offset into usage
             * 4: no. bytes to read
             *
             * RETURN
             * 1-4: echo command
             * 5+: data read
             */

            uint8_t usage_number    = commandBuf[1];
            uint8_t start_page      = commandBuf[2];
            uint8_t byte_offset     = commandBuf[3];
            uint8_t num_bytes       = commandBuf[4];

            int8_t usage_table_idx = FindUsageInTable(usage_number);

            if (usage_table_idx < 0)
            {
                pResponse[1] = UNKNOWN_USAGENUM;
                pResponse[2] = READWRITE_ERROR;
            }
            else
            {
                memcpy(&pResponse[0], &commandBuf[0], GENERIC_IN_PACKET_SIZE); // Put command in as echo response

                // Calculate how long the usage in in bytes
                usagetableentry_st *usage_info = GetUsageInfo(usage_table_idx);

                // Host can read 0 bytes --> indicates they want to read the usage table entry, only care about usage_number in this case
                if (num_bytes == 0)
                {
                    pResponse[4] = USAGE_ENTRY_SIZE; // Number of bytes 'read' (in fact just the size of a usage entry in this case)
                    memcpy(&pResponse[5], (uint8_t *)usage_info, sizeof(usagetableentry_st));
                }
                else
                {
                    // Check for errors in request
                    uint8_t check_for_errors = ReadWriteUsageErrorChecks(commandBuf, pResponse, usage_info);

                    // If FAIL don't do anything, error codes have been setup in ReadWriteUsageErrorChecks()
                    if (check_for_errors == PASS)
                    {
                        uint16_t start_address = ((usage_info->maxoffset & 0x80) == 0x00) ?
                                            ((uint16_t)usage_info->startpage << 8) + (((uint16_t)start_page * ((uint16_t)(usage_info->maxoffset & 0x7F) + 1)) * 2) + byte_offset :
                                            ((uint16_t)usage_info->startpage << 8) + (((uint16_t)start_page * 2) + byte_offset);

                        // Although it would be smart for the bridge to send multiple USB response packets if more than an endpoints worth has been requested, previous bridges
                        // have not done this. In order to maintain backwards compatibility with hosts/software it will be assumed the host is smart enough to split up the reads itself.
                        if (num_bytes > MAX_NUMBYTES_PER_CMDWRITEREAD)
                        {
                            num_bytes = MAX_NUMBYTES_PER_CMDWRITEREAD;
                        }

                        ReadAxiom(start_address, &pResponse[5], num_bytes);
                    }
                }
            }

            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_FIND_I2C_ADDRESS:
        {
            en_comms mode = GetCommsMode();

            pResponse[0] = (uint8_t)CMD_FIND_I2C_ADDRESS;

            if (mode == eSPI)
            {
                pResponse[1] = SPI_MODE_ADDRESS;
            }
            else if (mode == eI2C)
            {
                uint32_t i2c_addr = GetAxiomI2CAddress() >> 1U; // Address is stored in i2c format (i.e. lsb is the RW bit)

                if (i2c_addr == 0U)
                {
                    // An address of 0 indicates that no axiom is connected
                    pResponse[1] = (uint8_t)I2C_ERROR;
                }
                else
                {
                    pResponse[1] = (uint8_t)i2c_addr;
                }
            }
            else
            {
                // No comms selected - return error
                pResponse[1] = I2C_ERROR;
            }

            response_required = RESPONSE_REQUIRED;
            break;
        }

//------------------------------------------------------------Mode switch Commands------------------------------------------------------------

        case CMD_RESET_BRIDGE:
        {
            RestartBridge(udev);
            break;
        }
//-------
        case CMD_GET_PART_ID:
        {
            pResponse[0] = CMD_GET_PART_ID;
            pResponse[1] = ID_GDF350;

            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_ENTER_BOOTLOADER:
        {
            if ((commandBuf[1] == 0xAAu) && (commandBuf[2] == 0x55u) && (commandBuf[3] == 0xA5u) && (commandBuf[4] == 0x5Au))
            {
                EnterBootloader(udev);
            }

            break;
        }
//-------
        case CMD_GET_BRIDGE_MODE:
        {
            pResponse[0] = CMD_GET_BRIDGE_MODE;
            pResponse[1] = ReadBridgeMode();
            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_SWITCH_MODE_TBP_BASIC:
        {
            if(commandBuf[1] == MODE_SWITCH_OK)
            {
                WriteBridgeMode(udev, MODE_TBP_BASIC);
            }
            else
            {
                pResponse[0] = CMD_SWITCH_MODE_TBP_BASIC;
                pResponse[1] = INVALID_SETTINGS;
                response_required = RESPONSE_REQUIRED;
            }

            break;
        }
//-------
        case CMD_SWITCH_MODE_TBP_ABS_MOUSE:
        {
            if(commandBuf[1] == MODE_SWITCH_OK)
            {
                WriteBridgeMode(udev, MODE_ABSOLUTE_MOUSE);
            }
            else
            {
                pResponse[0] = CMD_SWITCH_MODE_TBP_BASIC;
                pResponse[1] = INVALID_SETTINGS;
                response_required = RESPONSE_REQUIRED;
            }

            break;
        }
//-------
        case CMD_SWITCH_MODE_TBP_DIGITIZER:
        {
            if(commandBuf[1] == MODE_SWITCH_OK)
            {
                WriteBridgeMode(udev, MODE_PARALLEL_DIGITIZER);
            }
            else
            {
                pResponse[0] = CMD_SWITCH_MODE_TBP_BASIC;
                pResponse[1] = INVALID_SETTINGS;
                response_required = RESPONSE_REQUIRED;
            }

            break;
        }
//-------
        case CMD_BLOCK_DIGITIZER_REPORTS:
        {
            if (commandBuf[1] == 0x00)
            {
                BlockDigitizerPackets(eDigitizerPacketsAllowed);
            }
            else
            {
                BlockDigitizerPackets(eDigitizerPacketsBlocked);
            }

            // re-instate the cached proxy mode
            SetProxyMode(proxy_mode_cache);

            // Response is echo of command
            memcpy(pResponse, commandBuf, USBFS_MAX_PACKET_SIZE);

            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_BLOCK_PRESS_REPORTS:
        {
            if (commandBuf[1] == 0x00)
            {
                BlockPressPackets(ePressPacketsAllowed);
            }
            else
            {
                BlockPressPackets(ePressPacketsBlocked);
            }

            // re-instate the cached proxy mode
            SetProxyMode(proxy_mode_cache);

            // Response is echo of command
            memcpy(pResponse, commandBuf, USBFS_MAX_PACKET_SIZE);

            response_required = RESPONSE_REQUIRED;
            break;
        }

//------------------------------------------------------------TH2 Compatibility Commands------------------------------------------------------------

        case CMD_SET_CONFIG:
        {
            // COMPATIBILITY COMMAND
            pResponse[0] = CMD_SET_CONFIG;
            response_required = RESPONSE_REQUIRED;
            break;
        }
//-------
        case CMD_GET_CONFIG:
        {
            // COMPATIBILITY COMMAND
            pResponse[0] = CMD_GET_CONFIG;
            response_required = RESPONSE_REQUIRED;
            break;
        }

//------------------------------------------------------------Unknown Commands------------------------------------------------------------

        default:
        {
            // Unknown command - respond with command id and error status
            pResponse[0] = commandBuf[0];
            pResponse[1] = INVALID_COMMAND;
            response_required = RESPONSE_REQUIRED;
            break;
        }
    }

    return response_required;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static uint8_t ReadWriteUsageErrorChecks(uint8_t *pCommand, uint8_t *pResponse, usagetableentry_st *usage_info)
{
    uint16_t max_usage_length = ((usage_info->maxoffset & 0x80) == 0x00) ?
                                ((uint16_t)usage_info->numpages) * (((uint16_t)(usage_info->maxoffset & 0x7F) + 1) * 2) :
                                (((uint16_t)usage_info->numpages - 1) << 7) + (((uint16_t)(usage_info->maxoffset & 0x7F) + 1) * 2);
    uint8_t error_check_passed;

    if(usage_info->numpages == 0)
    {
        /* usage length zero --> usage is a report placeholder so can't be written to */
        pResponse[1] = 0x95;
        pResponse[2] = 0x80;    // error flag
        error_check_passed = FAIL;
    }
    else if(pCommand[2] > (usage_info->numpages - 1))
    {
        /* start page too large */
        pResponse[1] = 0x94;
        pResponse[2] = 0x80;    // error flag
        error_check_passed = FAIL;
    }
    else if(pCommand[3] >= max_usage_length)
    {
        /* offset into page address beyond end of usage */
        pResponse[1] = 0x93;
        pResponse[2] = 0x80;    // error flag
        error_check_passed = FAIL;
    }
    else if(pCommand[4] > max_usage_length)
    {
        /* too many bytes requested */
        pResponse[1] = 0x92;
        pResponse[2] = 0x80;    // error flag
        pResponse[3] = (uint8_t)(max_usage_length & 0xFF);
        pResponse[4] = (uint8_t)(max_usage_length >> 8);
        error_check_passed = FAIL;
    }
    else
    {
        // no errors found
        error_check_passed = PASS;
    }

    return error_check_passed;
}

