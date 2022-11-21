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
#include "generic_hid_itf.h"
#include "generic_hid_core.h"
#include "proxy_driver.h"

#include <string.h>

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/
static void GenericSetup(void);


/*******************************************************************************
 * Exported Variables
 ******************************************************************************/
generic_fop_handler g_generic_fops =
{
    .periph_config = GenericSetup,
    .GenericCommandReceived = 0
};

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
uint8_t g_ControlReportReady    = 0;
uint8_t g_circularBufferFill    = 0; // Data read from axiom is stored in the circular buffer at this index
uint8_t g_circularBufferEmpty   = 0; // Data sent to the host is read from this index

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define COMMS_OK    (0x00U)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void USBPacketReady(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief   Packages the report read from axiom as a proxy USB message to send to the host.
 * @param[in]   pReport Pointer to array containing the report read from axiom.
 * @param[out]  pMsg    Pointer to USB buffer where the packaged message will be copied to.
 * @return  None.
 */
void PrepareControlUSBReport(uint8_t *pReport, uint8_t *pMsg)
{
    // Make sure the USB buffer is empty
    memset(&pMsg[0], 0x00, GENERIC_IN_PACKET_SIZE);

    // Package report for Control interface
    memcpy(&pMsg[2], pReport, MAX_AXIOM_REPORT_LEN);
    pMsg[0] = PROXY_FLAG;
    pMsg[1] = COMMS_OK;
    USBPacketReady();
    MoveCircularBufferHead();
}

/**
 * @brief   Returns a flag indicating if a USB packet is ready to send.
 */
uint8_t CheckForControlUSBPacket(void)
{
    if (g_ControlReportReady == CONTROL_USB_PACKET_READY)
    {
        return CONTROL_USB_PACKET_READY;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief   Clears the flag marking a USB packet is ready to send.
 */
void ClearControlUSBPacketReady(void)
{
    g_ControlReportReady = 0;
}

void MoveCircularBufferHead(void)
{
    g_circularBufferFill++;
    g_circularBufferFill %= MAX_NUM_CONTROL_BUFFERS;
}

uint8_t GetCircularBufferHead(void)
{
    return g_circularBufferFill;
}

void MoveCircularBufferTail(void)
{
    g_circularBufferEmpty++;
    g_circularBufferEmpty %= MAX_NUM_CONTROL_BUFFERS;
}

uint8_t GetCircularBufferTail(void)
{
    return g_circularBufferEmpty;
}
/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void GenericSetup(void)
{
    g_generic_fops.GenericCommandReceived = 0U;
}

static void USBPacketReady(void)
{
    g_ControlReportReady = 1U;
}
