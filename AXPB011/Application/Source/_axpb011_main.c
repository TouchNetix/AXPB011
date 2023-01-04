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
#include <string.h>
#include "gd32f3x0.h"
#include "systick.h"
#include "init.h"
#include "command_processor.h"
#include "composite_usb_wrapper.h"
#include "comms.h"
#include "proxy_driver.h"
#include "crc_checksum.h"
#include "mode_control.h"
#include "timers_and_leds.h"

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
usb_core_driver g_composite_hid_device;

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/
#define TOUCH_REPORT            (0x41U)

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
static void ProcessControlCommand(usb_core_driver *pDevice);
static void ProcessPressCommand(usb_core_driver *pDevice, uint8_t *usbPressBuffer);
static void RunProxy(usb_dev *udev, uint8_t *USBControl, uint8_t *USBPress, uint8_t *USBDigitizer);
static void SendControlUSBMessage(usb_core_driver *pDevice, uint8_t *usbControlBuffer);
static void SendPressUSBMessage(usb_core_driver *pDevice, uint8_t *usbPressBuffer);
static void SendDigitizerUSBMessage(usb_core_driver *pDevice, uint8_t *usbDigitizerBuffer);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
int main(void)
{
    uint8_t ControlUSBReport[MAX_NUM_CONTROL_BUFFERS][GENERIC_IN_PACKET_SIZE]  = {0};
    uint8_t PressUSBReport[PRESS_IN_PACKET_SIZE]            = {0};
    uint8_t DigitizerUSBReport[DIGITIZER_IN_PACKET_SIZE]    = {0};

    if (DeviceInit(&g_composite_hid_device) == USB_HOST_ABSENT)
    {
        for (;;)
        {
            // Do nothing, forever.
            continue;
        }
    }

    for (;;)
    {
//-------------------------------CHECK FOR COMMANDS FROM HOST-------------------------------//
        if (g_generic_fops.GenericCommandReceived == CONTROL_COMMAND_RECEIVED)
        {
            ProcessControlCommand(&g_composite_hid_device);
        }

        if (g_press_fops.PressCommandReceived == PRESS_COMMAND_RECEIVED)
        {
            ProcessPressCommand(&g_composite_hid_device, PressUSBReport);
        }


//-------------------------------RUN PROXY MODE-------------------------------//
        RunProxy(&g_composite_hid_device, ControlUSBReport[GetCircularBufferHead()], PressUSBReport, DigitizerUSBReport);


//-------------------------------SEND USB PACKETS-------------------------------//
        // Don't send USB packets if host is in sleep state
        if (g_composite_hid_device.dev.pm.dev_remote_wakeup == 0U)
        {
            if (CheckForControlUSBPacket() == CONTROL_USB_PACKET_READY)
            {
                SendControlUSBMessage(&g_composite_hid_device, ControlUSBReport[GetCircularBufferTail()]);
            }

            if (CheckForPressUSBPacket() == PRESS_USB_PACKET_READY)
            {
                SendPressUSBMessage(&g_composite_hid_device, PressUSBReport);
            }

            if (CheckForDigitizerUSBPacket() == DIGITIZER_USB_PACKET_READY)
            {
                SendDigitizerUSBMessage(&g_composite_hid_device, DigitizerUSBReport);
            }
        }

        ControlLEDs();

        // Prevents reading axiom before it has time to de-assert nIRQ
        // Time between EOT and nIRQ de-asserting = ~22us
        // Sleep to allow nIRQ time to de-assert.
        delay_1us(30);
    }
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void ProcessControlCommand(usb_core_driver *pDevice)
{
    // Don't add it to the circular buffer, needs a response immediately
    uint8_t responseBuffer[GENERIC_IN_PACKET_SIZE] = {0};

    if (ProcessTBPCommand(&g_composite_hid_device, responseBuffer, CONTROL_INTERFACE) == RESPONSE_REQUIRED)
    {
        // Wait until the response can be sent
        while (generic_hid_report_send(&g_composite_hid_device, responseBuffer, GENERIC_IN_PACKET_SIZE) == USBD_BUSY)
        {
            continue;
        }

        memset(responseBuffer, 0x00, GENERIC_IN_PACKET_SIZE);
    }

    g_generic_fops.GenericCommandReceived = NO_CONTROL_COMMAND;
}

static void ProcessPressCommand(usb_core_driver *pDevice, uint8_t *usbPressBuffer)
{
    if (ProcessTBPCommand(&g_composite_hid_device, usbPressBuffer, PRESS_INTERFACE) == RESPONSE_REQUIRED)
    {
        // Wait until the response can be sent
        while (press_hid_report_send(&g_composite_hid_device, usbPressBuffer, PRESS_IN_PACKET_SIZE) == USBD_BUSY)
        {
            continue;
        }

        memset(usbPressBuffer, 0x00, PRESS_IN_PACKET_SIZE);
    }

    g_press_fops.PressCommandReceived = NO_PRESS_COMMAND;
}

static void RunProxy(usb_dev *udev, uint8_t *USBControl, uint8_t *USBPress, uint8_t *USBDigitizer)
{
    uint8_t ProxyReport[U34_PROXYLENGTH] = {0};

    switch (GetCurrentProxyMode())
    {
        // Sends reports on all interfaces (digitizer only if enabled)
        case eControlPressDigiProxy:
        {
            if (ReadProxyReport(ProxyReport) == eReportRead)
            {
                PrepareControlUSBReport(ProxyReport, USBControl);

                // Only prepare press and digitizer if report is a u41 and crc checksum passes.
                // Note we still send failed CRC reports to the control endpoint.
                if ((ProxyReport[1] == TOUCH_REPORT) && (CRC_Checksum(ProxyReport) == eCRC_Pass))
                {
                    PreparePressUSBReport(ProxyReport, USBPress);

                    if ((ReadBridgeMode() == MODE_PARALLEL_DIGITIZER) || (ReadBridgeMode() == MODE_ABSOLUTE_MOUSE))
                    {
                        PrepareDigitizerUSBReport(udev, ProxyReport, USBDigitizer);
                    }
                }
            }

            break;
        }

        // Default mode after a reset - reports sent on press and (if enabled) digitizer endpoint
        case ePressDigiProxy:
        {
            if (ReadProxyReport(ProxyReport) == eReportRead)
            {
                // Only prepare press and digitizer if report is a u41 and crc checksum passes.
                if ((ProxyReport[1] == TOUCH_REPORT) && (CRC_Checksum(ProxyReport) == eCRC_Pass))
                {
                    PreparePressUSBReport(ProxyReport, USBPress);

                    if ((ReadBridgeMode() == MODE_PARALLEL_DIGITIZER) || (ReadBridgeMode() == MODE_ABSOLUTE_MOUSE))
                    {
                        PrepareDigitizerUSBReport(udev, ProxyReport, USBDigitizer);
                    }
                }
            }

            break;
        }

        // Performing a multipage read (e.g. for 3D/diagnostic data)
        case eMultiPageRead:
        {
            enMultiPageStatus status = MultiPageProxyReport(ProxyReport);

            // Do a read from axiom (multipage style)
            if (status == eDataRead)
            {
                // Data read, queue packet for control USB interface
            }
            else if (status == eLastRead)
            {
                // Nothing left to read, turn proxy mode off and send last USB packet
                SetProxyMode(eProxyStop);
            }
            else
            {
                // Error - stop the MP read and send USB response
                SetProxyMode(eProxyStop);
                ProxyReport[0] = PROXY_FLAG;
                ProxyReport[1] = 0x01U;
            }

            PrepareControlUSBReport(ProxyReport, USBControl);

            break;
        }

        case eProxyStop:
        default:
        {
            break;
        }
    }
}

static void SendControlUSBMessage(usb_core_driver *pDevice, uint8_t *usbControlBuffer)
{
    if (GetCurrentProxyMode() == eMultiPageRead)
    {
        // Different procedure if in multipage read mode - don't want to drop any 3D data packets
        while (generic_hid_report_send(&g_composite_hid_device, usbControlBuffer, GENERIC_IN_PACKET_SIZE) == USBD_BUSY)
        {
            continue;
        }

        memset(usbControlBuffer, 0x00, GENERIC_IN_PACKET_SIZE);
        MoveCircularBufferTail();

        if (GetCircularBufferHead() == GetCircularBufferTail())
        {
            // No data left to send, clear the flag
            ClearControlUSBPacketReady();
        }
    }
    else
    {
        // If we're reading proxy reports faster than they can be sent to the host, the circular buffer head will overtake the tail
        if (generic_hid_report_send(&g_composite_hid_device, usbControlBuffer, GENERIC_IN_PACKET_SIZE) == USBD_OK)
        {
            memset(usbControlBuffer, 0x00, GENERIC_IN_PACKET_SIZE);
            MoveCircularBufferTail();

            if (GetCircularBufferHead() == GetCircularBufferTail())
            {
                // No data left to send, clear the flag
                ClearControlUSBPacketReady();
            }
        }
    }
}

static void SendPressUSBMessage(usb_core_driver *pDevice, uint8_t *usbPressBuffer)
{
    // If USB interface is busy we don't want to clear the packet
    if (press_hid_report_send(&g_composite_hid_device, usbPressBuffer, PRESS_IN_PACKET_SIZE) == USBD_OK)
    {
        memset(usbPressBuffer, 0x00, PRESS_IN_PACKET_SIZE);
        ClearPressUSBPacketReady();
    }
}

static void SendDigitizerUSBMessage(usb_core_driver *pDevice, uint8_t *usbDigitizerBuffer)
{
    // If USB interface is busy we don't want to clear the packet
    if (digitizer_hid_report_send(&g_composite_hid_device, usbDigitizerBuffer, GetDigitizerReportLength()) == USBD_OK)
    {
        memset(usbDigitizerBuffer, 0x00, DIGITIZER_IN_PACKET_SIZE);
        ClearDigitizerUSBPacketReady();
    }
}
