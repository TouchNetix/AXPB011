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

/* Sentry Guard Start */
#ifndef APPLICATION_INCLUDE_USAGE_CONTROLS_H_
#define APPLICATION_INCLUDE_USAGE_CONTROLS_H_

/*******************************************************************************
 * Include Directives
 ******************************************************************************/
#include "gd32f3x0.h"

/*******************************************************************************
 * Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * Data Types
 ******************************************************************************/
typedef struct
{
    uint8_t usagenum;
    uint8_t startpage;
    uint8_t numpages;
    uint8_t maxoffset;
    uint8_t uifrevision;
    uint8_t ufwrevision;
} usagetableentry_st;

typedef struct
{
    uint8_t     SemaphoreUsageNumber;
    uint8_t     SemaphoreOffset;
    uint8_t     SemaphoreStartByte;
    uint8_t     SemaphoreStopByte;
    uint8_t     NumTx;  // Number of bytes to write to axiom
    uint16_t    TotalNumRX; // TOTAL no. bytes to read (concatenated into a 16 bit word)
    uint8_t     PageLength; // Length of a page (in case page size changes in firmware update)
    uint16_t    AddrStart;  // Multi-page start target address

    uint16_t    BytesRead;  // Keeps track of how many bytes have been read
    uint8_t     PagesMovedThrough;  // Allows us to keep track of how many reads have been performed
    uint8_t     BytesOffset;    // Allows us to keep track of how many reads have been performed
} MultiPageSetupTypeDef;

/*******************************************************************************
 * Constants (including enums)
 ******************************************************************************/
#define MAX_NUM_USAGES              (83U)   // This is assuming the maximum no. usages fits on 2 pages (may change with future hardware revisions/releases)
#define USAGE_ENTRY_SIZE            (6U)
#define MAX_U31_PAGE_LENGTH         (256U)
#define DEVICE_INFO_ADDRESS         (uint16_t)(0x0000U)
#define USAGE_TABLE_ADDRESS         (uint16_t)(0x0100U)
#define DEVICE_INFO_LENGTH          (12U)
#define NUM_USAGES_OFFSET           (10U)
#define USAGENUM_U34                (0x34U)
#define USAGENUM_U35                (0x35U)

#define SEMAPHORE_START             (1)
#define SEMAPHORE_STOP              (0)

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

 
/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
void ClearLocalUsageTable(void);
void SaveUsageInLocalTable(uint8_t usage_table_idx, uint8_t *usagetable_buffer, uint8_t usage_len);
int8_t FindUsageInTable(uint8_t requested_usage_num);
usagetableentry_st *GetUsageInfo(uint8_t index);
void StoreMultipageReadInfo(MultiPageSetupTypeDef *setup);
MultiPageSetupTypeDef *GetMultiPageReadInfo(void);
void WriteSemaphore(uint8_t start_stop);
uint16_t Getu34Addr(void);
void Updateu34Addr(uint16_t addr);
uint16_t Getu35Addr(void);
void Updateu35Addr(uint16_t addr);
						   
/*******************************************************************************
 * Macros
 ******************************************************************************/

 
/*******************************************************************************
 * Inline Functions
 ******************************************************************************/


/* END SENTRY GUARD */
#endif /* APPLICATION_INCLUDE_USAGE_CONTROLS_H_ */

