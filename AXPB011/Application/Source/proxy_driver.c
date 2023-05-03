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
#include <stdio.h>
#include <string.h>
#include "proxy_driver.h"
#include "comms.h"
#include "usage_controls.h"

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
#define NUMBYTES_RX_MP  (58U)

/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
static enBridgeProxyMode g_ProxyMode = eProxyStop;

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


/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief
 * @param[in]
 * @return None.
 */
void SetProxyMode(enBridgeProxyMode NewMode)
{
    g_ProxyMode = NewMode;
}

/**
 * @brief
 * @param[in]   None.
 * @param[out]  None.
 */
enBridgeProxyMode GetCurrentProxyMode(void)
{
    return g_ProxyMode;
}

/**
 * @brief   Reads a report from u34 and stores it in *pbuf
 * @param[out]  pbuf    Pointer to the buffer where the report will be written to
 * @return  Status of whether a report was read from axiom or not.
 * @retval  eReportRead Report successfully read from axiom.
 * @retval  eNoReport   No report could be read. Either axiom didn't respond or pbuf was NULL.
 */
enProxyStatus ReadProxyReport(uint8_t *pbuf)
{
    enProxyStatus status = eNoReport;

    if (pbuf != NULL)
    {
        if (gpio_input_bit_get(NIRQ_GPIO_PORT, NIRQ_PIN) == RESET)
        {
            memset(pbuf, 0x00, U34_PROXYLENGTH);

            // Read from u34 - no need to read exact length, can just read full report out each time
            ReadAxiom(Getu34Addr(), pbuf, U34_PROXYLENGTH);

            status = eReportRead;
        }
    }

    return status;
}

enMultiPageStatus MultiPageProxyReport(uint8_t *pbuf)
{
    enMultiPageStatus status = eNoReport;

    // Get a pointer to the multipage info structure
    MultiPageSetupTypeDef *pMPInfo = GetMultiPageReadInfo();

    // set no. read bytes to either 58 (maximum read size) or however many bytes are left
    // --> 3 bytes come from 'nonsense' at end of read(?)
    //     2 bytes for header (command and comms status)
    //     1 byte report ID
    uint16_t length = (pMPInfo->TotalNumRX < NUMBYTES_RX_MP) ? pMPInfo->TotalNumRX : NUMBYTES_RX_MP;

    uint16_t next_address = pMPInfo->AddrStart + (pMPInfo->PagesMovedThrough << 8U) + (pMPInfo->BytesOffset & 0xFFU);

    ReadAxiom(next_address, pbuf, length);

    // Keep track of how many bytes have been read
    pMPInfo->BytesRead += length;

    // calculate the next starting address and offset required (e.g. start at 0, read 64, so new starting address is 0x0064)
    pMPInfo->PagesMovedThrough  = pMPInfo->BytesRead / (pMPInfo->PageLength == 0 ? (uint16_t)256 : (uint16_t)pMPInfo->PageLength);
    pMPInfo->BytesOffset        = pMPInfo->BytesRead % (pMPInfo->PageLength == 0 ? (uint16_t)256 : (uint16_t)pMPInfo->PageLength);

    if ((pMPInfo->TotalNumRX - pMPInfo->BytesRead) == 0)
    {
        // Nothing else to read
        status = eLastRead;
    }
    else
    {
        status = eDataRead;
    }

    return status;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/

