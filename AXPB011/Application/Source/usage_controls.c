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
#include "usage_controls.h"
#include "systick.h"
#include "comms.h"

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

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define MAX_RETRY_NUM   (200U)

/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
static usagetableentry_st g_usagetable[MAX_NUM_USAGES] = {0};
static MultiPageSetupTypeDef g_MultipageReadInfo = {0};
static uint16_t g_u34_addr = 0;

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static uint8_t GetSemaphoreStartStop(uint8_t start_stop);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
void ClearLocalUsageTable(void)
{
    memset((uint8_t *)g_usagetable, 0, sizeof(g_usagetable));
}

void SaveUsageInLocalTable(uint8_t usage_table_idx, uint8_t *pusagetable_buffer, uint8_t usage_len)
{
    memcpy((uint8_t *)&g_usagetable[usage_table_idx], &pusagetable_buffer[0], usage_len);

    // Store some key usages for faster access
    if (g_usagetable[usage_table_idx].usagenum == USAGENUM_U34)
    {
        Updateu34Addr((g_usagetable[usage_table_idx].startpage & 0xFF) << 8);
    }
}

/**
 * @brief   Returns the index of the requested usage in the local usage table.
 * @param[in]   requested_usage_num Usage number in hex.
 * @return  Index in the local usage table.
 * @retval  -1  Usage not found.
 */
int8_t FindUsageInTable(uint8_t requested_usage_num)
{
    int8_t index = -1;

    for(uint8_t usage_table_idx = 0; usage_table_idx < MAX_NUM_USAGES; usage_table_idx++)
    {
        if(g_usagetable[usage_table_idx].usagenum == requested_usage_num)
        {
            index = usage_table_idx;
        }
    }

    return index;
}

usagetableentry_st *GetUsageInfo(uint8_t index)
{
    return &g_usagetable[index];
}

uint16_t Getu34Addr(void)
{
    return g_u34_addr;
}

void Updateu34Addr(uint16_t addr)
{
    g_u34_addr = addr;
}

void StoreMultipageReadInfo(MultiPageSetupTypeDef *pSetup)
{
    memcpy(&g_MultipageReadInfo, pSetup, sizeof(MultiPageSetupTypeDef));
}

/**
 * @brief   Returns a pointer to the Multipage setup information.
 */
MultiPageSetupTypeDef *GetMultiPageReadInfo(void)
{
    return &g_MultipageReadInfo;
}

/**
 * @brief   Write semaphore start or stop byte to the desired usage.
 * @param[in]   semaphore_page_num  Page number where the semaphore is to be written to.
 * @param[in]   semaphore_offset    Offset into page where the semaphore is to be written to.
 * @param[in]   start_stop  Indicates if we're sending the start or stop byte.
 */
void WriteSemaphore(uint8_t start_stop)
{
    int8_t usage_idx = FindUsageInTable(g_MultipageReadInfo.SemaphoreUsageNumber);

    // Set the address of the usage we want to write the semaphore to
    uint16_t target_address = ((g_usagetable[usage_idx].startpage << 8) & 0xFF00) & (g_MultipageReadInfo.SemaphoreOffset & 0xFF);

    // Send the start or stop byte (determined by function parameter)
    uint8_t sempahore_byte_write = 0;

    if (start_stop == SEMAPHORE_START)
    {
        sempahore_byte_write = GetSemaphoreStartStop(SEMAPHORE_START);
    }
    else
    {
        sempahore_byte_write = GetSemaphoreStartStop(SEMAPHORE_STOP);
    }

    WriteAxiom(target_address, &sempahore_byte_write, 1);

    // Wait for the change to happen in axiom
    uint8_t sempahore_byte_read = 0;
    uint8_t retry = 0;
    while ((sempahore_byte_read != sempahore_byte_write) && (retry < MAX_RETRY_NUM))
    {
        delay_1ms(10);
        ReadAxiom(target_address, &sempahore_byte_read, 1);

        retry++;
    }
}
/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static uint8_t GetSemaphoreStartStop(uint8_t start_stop)
{
    uint8_t semaphore = 0;

    if (start_stop == SEMAPHORE_START)
    {
        semaphore = g_MultipageReadInfo.SemaphoreStartByte;
    }
    else
    {
        semaphore = g_MultipageReadInfo.SemaphoreStopByte;
    }

    return semaphore;
}
