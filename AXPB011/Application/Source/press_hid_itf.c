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
#include "press_hid_itf.h"
#include "press_hid_core.h"

#include <string.h>

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/
struct st_PressTouchData
{
    uint8_t touch_present;
    uint8_t x_coord_msb;
    uint8_t x_coord_lsb;
    uint8_t y_coord_msb;
    uint8_t y_coord_lsb;
    uint8_t z_amp;
};

/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void press_setup(void);


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
press_fop_handler g_press_fops =
{
    .periph_config = press_setup,
    .PressCommandReceived = 0
};

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
uint8_t g_PressReportReady = 0;
enPressBlockedStatus g_AllowPressReports = ePressPacketsAllowed;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define TOUCH_IDS                       (2U)
#define NOT_USED                        (0x00)
#define IDFIELD_TOUCHSCREENDATA         (0x03)
#define IDFIELD_TOUCHXYDATA             (0x02)
#define IDFIELD_ENDOFLIST               (0x00)
#define PAYLOADLENGTH_TOUCHSCREENDATA   (2)
#define PAYLOADLENGTH_TOUCHXYDATA       (6)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void USBPacketReady(void);
static uint8_t GetNumberOfTouches(uint8_t TouchIDByte);
static void ExtractTouchDataFromReport(uint8_t touch_num, uint8_t *pBuf, struct st_PressTouchData *pTouchData);
static enPressBlockedStatus CheckIfPressUSBBlocked(void);

/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief   Packages the report read from axiom for the press endpoint.
 * @param[in]   pReport Pointer to array containing the report read from axiom.
 * @param[out]  pMsg    Pointer to USB buffer where the packaged message will be copied to.
 * @return  None.
 */
void PreparePressUSBReport(uint8_t *pReport, uint8_t *pMsg)
{
    if (CheckIfPressUSBBlocked() == ePressPacketsAllowed)
    {
        // Build press report
        uint8_t ByteCount = 0;

        /* In PB-005 we used to send the data from the 4 press channels first, then send each touch with associated XYZ coordinates --> down side of this was had to cut out region information from touches!
         * In this bridge we're only sending the touches and the associated Z amplitude (as it has already been calculated and processed in aXiom anyway).
         * Order of data (each one pre-ceeded by the corresponding ID):
         *  - No. total touches
         *  - Touch data (x5)
         *      --> TouchID (touch num. and if its present), X low, X high, Y low, Y high, Z amplitude
         *  - End of List marker
         */

        // 3rd byte of u41 report holds the current number of touches
        uint8_t num_touches = GetNumberOfTouches(pReport[TOUCH_IDS]);

        // Send overall touch screen status and no. total touches - Byte count starts at 0 and increments AFTER the line has been executed
        pMsg[ByteCount++] = ((PAYLOADLENGTH_TOUCHSCREENDATA << 3) | IDFIELD_TOUCHSCREENDATA);  // ID Field
        pMsg[ByteCount++] = num_touches;  // Payload
        pMsg[ByteCount++] = NOT_USED;  // Empty

        // Touch XYZ data
        for(uint8_t TouchIdx = 0; TouchIdx < 5; TouchIdx++)
        {
            struct st_PressTouchData TouchInfo = {0};
            ExtractTouchDataFromReport(TouchIdx, pReport, &TouchInfo);

            pMsg[ByteCount++] = (PAYLOADLENGTH_TOUCHXYDATA << 3 ) | IDFIELD_TOUCHXYDATA; // ID Field
            pMsg[ByteCount++] = TouchInfo.touch_present | TouchIdx;  // Status field --> whether touch is present and touch number

            // X and Y are 2 bytes long, Z is 1 byte long = 5 bytes in total
            pMsg[ByteCount++] = TouchInfo.x_coord_lsb;
            pMsg[ByteCount++] = TouchInfo.x_coord_msb;
            pMsg[ByteCount++] = TouchInfo.y_coord_lsb;
            pMsg[ByteCount++] = TouchInfo.y_coord_msb;
            pMsg[ByteCount++] = TouchInfo.z_amp;
        }

        // set to the EndOfList ID field to indicate the end of the packet
        pMsg[ByteCount] = IDFIELD_ENDOFLIST;

        // Mark usb packet ready to send
        USBPacketReady();
    }
}

/**
 * @brief   Returns a flag indicating if a USB packet is ready to send.
 */
uint8_t CheckForPressUSBPacket(void)
{
    if (g_PressReportReady == PRESS_USB_PACKET_READY)
    {
        return PRESS_USB_PACKET_READY;
    }
    else
    {
        return NO_PRESS_USB_PACKET;
    }
}

/**
 * @brief   Clears the flag marking a USB packet is ready to send.
 */
void ClearPressUSBPacketReady(void)
{
    g_PressReportReady = NO_PRESS_USB_PACKET;
}

/**
 * @brief   Sets whether the press interface is blocked from sending USB packets or not.
 * @param[in]   state   Controls whether press packets are blocked or allowed to send.
 * @return  None.
 */
void BlockPressPackets(enPressBlockedStatus state)
{
    g_AllowPressReports = state;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void press_setup(void)
{
    g_press_fops.PressCommandReceived = 0U;
}

static void USBPacketReady(void)
{
    g_PressReportReady = PRESS_USB_PACKET_READY;
}

static uint8_t GetNumberOfTouches(uint8_t TouchIDByte)
{
    uint8_t NumTouches = 0;

    NumTouches = ((TouchIDByte & 1)  ? 1 : 0)  +   // these bits indicate how many touches are present on the screen
                 ((TouchIDByte & 2)  ? 1 : 0)  +
                 ((TouchIDByte & 4)  ? 1 : 0)  +
                 ((TouchIDByte & 8)  ? 1 : 0)  +
                 ((TouchIDByte & 16) ? 1 : 0);

    return NumTouches;
}

static void ExtractTouchDataFromReport(uint8_t touch_num, uint8_t *pBuf, struct st_PressTouchData *pTouchData)
{
    pTouchData->touch_present   = ((pBuf[2] | (pBuf[3] << 8)) & (1 << touch_num)) ? 0x20 : 0x00;
    pTouchData->x_coord_lsb     = (uint8_t)pBuf[4  + (touch_num * 4)];
    pTouchData->x_coord_msb     = (uint8_t)pBuf[5  + (touch_num * 4)];
    pTouchData->y_coord_lsb     = (uint8_t)pBuf[6  + (touch_num * 4)];
    pTouchData->y_coord_msb     = (uint8_t)pBuf[7  + (touch_num * 4)];
    pTouchData->z_amp           = (uint8_t)pBuf[44 + (touch_num * 1)];
}

static enPressBlockedStatus CheckIfPressUSBBlocked(void)
{
    return g_AllowPressReports;
}
