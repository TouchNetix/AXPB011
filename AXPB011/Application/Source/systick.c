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
#include "gd32f3x0.h"
#include "systick.h"

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
 * File Scope Variables
 ******************************************************************************/
volatile static uint32_t g_delay;

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
/*!
 \brief      configure systick
 \param[in]  none
 \param[out] none
 \retval     none
 */
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        /* capture error */
        while (1)
        {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}


/*!
 \brief      delay a time in milliseconds
 \param[in]  count: count in milliseconds
 \param[out] none
 \retval     none
 */
void delay_1ms(uint32_t count)
{
    g_delay = count;

    // delay is decremented as part of SysTick_Handler() call
    while (0U != g_delay)
        ;
}


/**
 * @brief   Delays program for desired amount.
 * @note    Due to loop overhead, the delay time may be slightly longer.
 * @param[in]   delay   Time in micro-seconds the program should wait for.
 */
void delay_1us(uint32_t delay)
{
    for (uint32_t wait = 0; wait < (delay * 10); wait++)
    {
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
    }
}

/*!
 \brief      delay decrement
 \param[in]  none
 \param[out] none
 \retval     none
 */
void delay_decrement(void)
{
    if (0U != g_delay)
    {
        g_delay--;
    }
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
