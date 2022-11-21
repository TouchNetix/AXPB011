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
#include "digitizer_hid_itf.h"
#include "digitizer_hid_core.h"
#include "mode_control.h"
#include "gd32f3x0_it.h"

#include <string.h>

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/
struct st_PressTouchData
{
    uint8_t     touch_present;
    uint16_t    x_coord;
    uint16_t    y_coord;
    uint8_t     z_amp;
};

/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void digitizer_setup(void);
static void ProcessMultipointDigitizer(usb_dev *udev, uint8_t *pReport, uint8_t *pMsg);
static void ProcessMouseDigitizer(usb_dev *udev, uint8_t *pReport, uint8_t *pMsg);
static void PrepareAbsMouseReport(uint8_t *pBuf, struct st_PressTouchData TouchInfo, uint8_t click_state);
static uint8_t GetNumberOfTouches(uint8_t TouchIDByte);
static enDigitizerBlockedStatus CheckIfDigitizerUSBBlocked(void);
static void ExtractTouchDataFromReport(uint8_t touch_num, uint8_t *pBuf, struct st_PressTouchData *pTouchData);
static void USBPacketReady(void);

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
digitizer_fop_handler g_digitizer_fops =
{
    .periph_config = digitizer_setup,
};


/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
uint8_t g_digitizerReportReady = 0;
volatile uint16_t g_digiTick = 0;
enDigitizerBlockedStatus g_allowDigitizerReports = eDigitizerPacketsAllowed;
en_HostWakeOptions g_hostWakeOption = eWakeOnTouch;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define REPORT_ID                   (1U)    // Relates to the report number found in the report descriptor (allows windows to differentiate between different connected devices)
#define CONFIDENCE                  (0x04U)
#define TIP_SWITCH                  (0x01U)
#define IN_RANGE                    (0x02U)
#define DATABYTES_PER_TOUCH         (7U)
#define TOUCH_NUMBER                (1U)
#define X_COORD_LSB                 (2U)
#define X_COORD_MSB                 (3U)
#define Y_COORD_LSB                 (4U)
#define Y_COORD_MSB                 (5U)
#define PRESSURE_LSB                (6U)
#define PRESSURE_MSB                (7U)

#define TOUCH_IDS                   (2U)
#define LEFT_BUTTON_PRESS           (0x01U)
#define LEFT_BUTTON_RELEASE         (0x00U)
#define RIGHT_BUTTON_PRESS          (0x02U)
#define RIGHT_BUTTON_RELEASE        (0x00U)

#define NO_WAKE                     (0x00U)
#define WAKE_ON_TOUCH               (0x01U)
#define WAKE_ON_TOUCH_HOVER         (0x03U)
#define WAKE_ON_TOUCH_HOVER_PROX    (0x07U)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/
#define ALIGN_WITH_CORRECT_TOUCH(x) ( (x - 1) * DATABYTES_PER_TOUCH )


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void USBPacketReady(void);
static void ExtractTouchDataFromReport(uint8_t touch_num, uint8_t *pBuf, struct st_PressTouchData *pTouchData);
static enDigitizerBlockedStatus CheckIfDigitizerUSBBlocked(void);
static void ProcessHostWake(usb_dev *udev, uint8_t num_touches, struct st_PressTouchData *pTouchInfo);

/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief   Packages the report read from axiom as a windows USB touch message.
 * @param[in]   pReport Pointer to array containing the report read from axiom.
 * @param[out]  pMsg    Pointer to USB buffer where the packaged message will be copied to.
 * @return  None.
 */
void PrepareDigitizerUSBReport(usb_dev *udev, uint8_t *pReport, uint8_t *pMsg)
{
    if (CheckIfDigitizerUSBBlocked() == eDigitizerPacketsAllowed)
    {
        switch (ReadBridgeMode())
        {
            case MODE_PARALLEL_DIGITIZER:
            {
                ProcessMultipointDigitizer(udev, pReport, pMsg);
                USBPacketReady();
                break;
            }

            case MODE_ABSOLUTE_MOUSE:
            {
                ProcessMouseDigitizer(udev, pReport, pMsg);
                USBPacketReady();
                break;
            }

            case MODE_TBP_BASIC:
            default:
            {
                // Do nothing - Digitizer endpoint is not active
                break;
            }
        }
    }
}

/**
 * @brief   Increments the digitizer timestamp.
 */
void IncrememtDigitizerTimeStamp(void)
{
    // Wrapping doesn't matter, Windows expects it to
    g_digiTick++;
}

/**
 * @brief   Returns a flag indicating if a USB packet is ready to send.
 */
uint8_t CheckForDigitizerUSBPacket(void)
{
    if (g_digitizerReportReady == DIGITIZER_USB_PACKET_READY)
    {
        return DIGITIZER_USB_PACKET_READY;
    }
    else
    {
        return NO_DIGITIZER_USB_PACKET;
    }
}

/**
 * @brief   Clears the flag marking a USB packet is ready to send.
 */
void ClearDigitizerUSBPacketReady(void)
{
    g_digitizerReportReady = NO_DIGITIZER_USB_PACKET;
}

/**
 * @brief   Sets whether the digitizer interface is blocked from sending USB packets or not.
 * @param[in]   state   Controls whether digitizer packets are blocked or allowed to send.
 * @return  None.
 */
void BlockDigitizerPackets(enDigitizerBlockedStatus state)
{
    g_allowDigitizerReports = state;
}

/**
 * @brief   Returns the length of the digitizer report, depending on current mode
 */
uint32_t GetDigitizerReportLength(void)
{
    uint32_t length = 0;

    switch (ReadBridgeMode())
    {
        case MODE_PARALLEL_DIGITIZER:
        {
            length = DIGITIZER_IN_PACKET_SIZE;
            break;
        }

        case MODE_ABSOLUTE_MOUSE:
        {
            length = MOUSE_ABS_IN_PACKET_SIZE;
            break;
        }

        default:
        case MODE_TBP_BASIC:
        {
            length = 0;
            break;
        }
    }

    return length;
}

/**
 * @brief   Sets what type of touch event will wake the host from sleep.
 * @param[in]   wake_option
 *  @arg    eNoWake
 *  @arg    eWakeOnTouch
 *  @arg    eWakeOnTouchHover
 *  @arg    eWakeOnTouchHoverProx
 */
void SetHostWakeOption(en_HostWakeOptions wake_option)
{
    switch (wake_option)
    {
        case 0x00U:
        {
            g_hostWakeOption = eNoWake;
            break;
        }

        case 0x01U:
        {
            g_hostWakeOption = eWakeOnTouch;
            break;
        }

        case 0x03U:
        {
            g_hostWakeOption = eWakeOnTouchHover;
            break;
        }

        case 0x07U:
        {
            g_hostWakeOption = eWakeOnTouchHoverProx;
            break;
        }

        default:
        {
            g_hostWakeOption = eWakeOnTouch;
        }
    }
    g_hostWakeOption = wake_option;
}

/**
 * @brief   Returns what type of touch event will wake the host from sleep.
 */
en_HostWakeOptions GetHostWakeOption(void)
{
    return g_hostWakeOption;
}
/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void digitizer_setup(void)
{
    // Nothing to do.
}

static void ProcessMultipointDigitizer(usb_dev *udev, uint8_t *pReport, uint8_t *pMsg)
{
    if (udev->dev.pm.dev_remote_wakeup == 1U)
    {
        struct st_PressTouchData FirstTouchInfo = {0};
        ExtractTouchDataFromReport(0, pReport, &FirstTouchInfo);

        ProcessHostWake(udev, GetNumberOfTouches(pReport[TOUCH_IDS]), &FirstTouchInfo);
    }
    else // else means digitizer reports aren't processed
    {
        pMsg[0] = REPORT_ID;

        for (uint8_t TouchNum = 0U; TouchNum < MAX_NUM_DIGITIZER_TOUCHES; TouchNum++)
        {
            struct st_PressTouchData TouchInfo = {0};
            uint8_t touch_state = 0;
            ExtractTouchDataFromReport(TouchNum, pReport, &TouchInfo);

            touch_state = CONFIDENCE;   // Windows requires this to be set even when no touches are present

            if (TouchInfo.touch_present)
            {
                // Touch detected
                // If z coordinate is a negative value it indicates there is a hover or prox
                // Note prox will trigger a hover event in Windows at the centre of the screen, this is by design.
                // If not desired the proximity threshold can be set below the hover threshold, such that it will never trigger
                if (TouchInfo.z_amp >= 0x80)
                {
                    // Hover or prox
                    touch_state |= IN_RANGE;
                }
                else
                {
                    // Touch
                    touch_state |= IN_RANGE | TIP_SWITCH;
                }
            }

            // Scales Z amplitude between 0-1024
            uint16_t digitizer_pressure = TouchInfo.z_amp;
            digitizer_pressure += 1;
            digitizer_pressure *= 4;

            // Windows XY coordinate range is 0-4095, hence the right-shift (axiom range is 0-65535 -> RS by 4 to obtain 4095).
            // Loses resolution but necessary for Windows.
            pMsg[ALIGN_WITH_CORRECT_TOUCH(TouchNum + 1) + TOUCH_NUMBER]      = ((TouchNum << 3U) | touch_state);
            pMsg[ALIGN_WITH_CORRECT_TOUCH(TouchNum + 1) + X_COORD_LSB]       = (uint8_t)((TouchInfo.x_coord >> 4U) & 0xFF);
            pMsg[ALIGN_WITH_CORRECT_TOUCH(TouchNum + 1) + X_COORD_MSB]       = (uint8_t)((TouchInfo.x_coord >> 4U) >> 8);
            pMsg[ALIGN_WITH_CORRECT_TOUCH(TouchNum + 1) + Y_COORD_LSB]       = (uint8_t)((TouchInfo.y_coord >> 4U) & 0xFF);
            pMsg[ALIGN_WITH_CORRECT_TOUCH(TouchNum + 1) + Y_COORD_MSB]       = (uint8_t)((TouchInfo.y_coord >> 4U) >> 8);
            pMsg[ALIGN_WITH_CORRECT_TOUCH(TouchNum + 1) + PRESSURE_LSB]      = (uint8_t) (digitizer_pressure & 0xFF);
            pMsg[ALIGN_WITH_CORRECT_TOUCH(TouchNum + 1) + PRESSURE_MSB]      = (uint8_t)((digitizer_pressure >> 8U) & 0xFF);
        }

        uint16_t digitizer_timer = g_digiTick;

        // Add timestamp to end of USB packet
        pMsg[(MAX_NUM_DIGITIZER_TOUCHES * DATABYTES_PER_TOUCH) + 1] = digitizer_timer & 0xFF;
        pMsg[(MAX_NUM_DIGITIZER_TOUCHES * DATABYTES_PER_TOUCH) + 2] = digitizer_timer >> 8;
        pMsg[(MAX_NUM_DIGITIZER_TOUCHES * DATABYTES_PER_TOUCH) + 3] = MAX_NUM_DIGITIZER_TOUCHES;   // Can be set between a value of 5 and 10 --> windows digitizer requires at least 5 touches to work correctly
    }
}

static void ProcessMouseDigitizer(usb_dev *udev, uint8_t *pReport, uint8_t *pMsg)
{
    static uint8_t num_touches_was = 0;
    static uint8_t num_touches_is = 0;

    if (udev->dev.pm.dev_remote_wakeup == 1U)
    {
        struct st_PressTouchData FirstTouchInfo = {0};
        ExtractTouchDataFromReport(0, pReport, &FirstTouchInfo);

        ProcessHostWake(udev, GetNumberOfTouches(pReport[TOUCH_IDS]), &FirstTouchInfo);
    }
    else // else means digitizer reports aren't sent until the host is awake
    {
        num_touches_was = num_touches_is;
        num_touches_is = GetNumberOfTouches(pReport[TOUCH_IDS]);

        // We only care about coordinates for the first touch - second touch (right click) can be anywhere on the screen
        struct st_PressTouchData touch1 = {0};
        ExtractTouchDataFromReport(0, pReport, &touch1);

        // If z is negative, reject the touch and do nothing (abs mouse doesn't support hover)
        if ((touch1.z_amp & 0x80) != 0x80)
        {
            if ((num_touches_is < 2) && (num_touches_was == 2))
            {
                // If first touch is still present we want it to persist through the right click
                if(touch1.touch_present)
                {
                    PrepareAbsMouseReport(pMsg, touch1, (LEFT_BUTTON_PRESS | RIGHT_BUTTON_RELEASE));
                }
                else
                {
                    PrepareAbsMouseReport(pMsg, touch1, (LEFT_BUTTON_RELEASE | RIGHT_BUTTON_RELEASE));
                }
            }
            else if ((num_touches_is == 0) && (num_touches_was > 0))
            {
                PrepareAbsMouseReport(pMsg, touch1, (LEFT_BUTTON_RELEASE | RIGHT_BUTTON_RELEASE));
            }
            else
            {
                if(touch1.touch_present)
                {
                    if (num_touches_is == 2)
                    {
                        PrepareAbsMouseReport(pMsg, touch1, (LEFT_BUTTON_PRESS | RIGHT_BUTTON_PRESS));
                    }
                    else
                    {
                        PrepareAbsMouseReport(pMsg, touch1, (LEFT_BUTTON_PRESS | RIGHT_BUTTON_RELEASE));
                    }
                }
            }
        }
    }
}

static void PrepareAbsMouseReport(uint8_t *pBuf, struct st_PressTouchData TouchInfo, uint8_t click_state)
{
    pBuf[0] = (0xF8 | click_state);
    pBuf[1] = (uint8_t)((TouchInfo.x_coord >> 4U) & 0xFF);
    pBuf[2] = (uint8_t)((TouchInfo.x_coord >> 4U) >> 8);
    pBuf[3] = (uint8_t)((TouchInfo.y_coord >> 4U) & 0xFF);
    pBuf[4] = (uint8_t)((TouchInfo.y_coord >> 4U) >> 8);
}

static uint8_t GetNumberOfTouches(uint8_t TouchIDByte)
{
    uint8_t NumTouches = 0;

    NumTouches = ((TouchIDByte & 1)  ? 1 : 0)  +
                 ((TouchIDByte & 2)  ? 1 : 0)  +
                 ((TouchIDByte & 4)  ? 1 : 0)  +
                 ((TouchIDByte & 8)  ? 1 : 0)  +
                 ((TouchIDByte & 16) ? 1 : 0);

    return NumTouches;
}

static void USBPacketReady(void)
{
    g_digitizerReportReady = DIGITIZER_USB_PACKET_READY;
}

static void ExtractTouchDataFromReport(uint8_t touch_num, uint8_t *pBuf, struct st_PressTouchData *pTouchData)
{
    uint8_t x_lsb, x_msb, y_lsb, y_msb;

    pTouchData->touch_present = ((uint16_t)(pBuf[2] | (pBuf[3] << 8)) & (uint16_t)(1 << touch_num)) ? 0x20 : 0x00;

    x_lsb = pBuf[4 + (touch_num*4)];
    x_msb = pBuf[5 + (touch_num*4)];
    pTouchData->x_coord = (uint16_t)(((x_msb) << 8U) | x_lsb);

    y_lsb = pBuf[6 + (touch_num*4)];
    y_msb = pBuf[7 + (touch_num*4)];
    pTouchData->y_coord = (uint16_t)(((y_msb) << 8U) | y_lsb);

    pTouchData->z_amp = (uint8_t)pBuf[44 + (touch_num * 1)];
}

static enDigitizerBlockedStatus CheckIfDigitizerUSBBlocked(void)
{
    return g_allowDigitizerReports;
}

static void ProcessHostWake(usb_dev *udev, uint8_t num_touches, struct st_PressTouchData *pTouchInfo)
{
    en_HostWakeOptions wake_method = GetHostWakeOption();

    // For host remote wake to work, the USB device needs to be allowed to wake the host in device manager (power management setting
    // under the touch screen interface/device). This worked on a deskptop, however it did not work on a laptop with a native touchscreen.
    // This could be some hardware setting to block a touch screen from waking the laptop, or maybe a different power management
    // setting.
    // This link has information on how to implement wake-on-touch for Windows 11 (doesn't seem to apply for Windows 10):
    // https://docs.microsoft.com/en-us/windows-hardware/design/component-guidelines/wake-on-touch-implementation-guide

    switch (wake_method)
    {
        case eNoWake:
        {
            // Do nothing.
            break;
        }

        case eWakeOnTouchHover:
        {
            // Z value must be greater than 0x80 to be classed as hover --> 0x80 indicates there is a prox event
            if ((num_touches > 0) || (pTouchInfo->z_amp > 0x80))
            {
                WakeHost(udev);
            }
            break;
        }

        case eWakeOnTouchHoverProx:
        {
            // z value can be greater than (hover) or equal to (prox) 0x80
            if ((num_touches > 0) || (pTouchInfo->z_amp >= 0x80))
            {
                WakeHost(udev);
            }
            break;
        }

        case eWakeOnTouch:
        default:
        {
            if ((num_touches > 0) && (pTouchInfo->z_amp <= 0x7F))
            {
                WakeHost(udev);
            }

            break;
        }
    }
}
