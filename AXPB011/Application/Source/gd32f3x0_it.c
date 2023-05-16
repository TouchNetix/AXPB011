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
#include "gd32f3x0_it.h"
#include "systick.h"
#include "usb_conf.h"
#include "drv_usbd_int.h"
#include "generic_hid_core.h"
#include "digitizer_hid_itf.h"
#include "init.h"
#include "proxy_driver.h"
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
extern uint32_t g_usbfs_prescaler;

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


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
void usb_timer_irq(void);
static void resume_mcu_clk(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/*!
 \brief      this function handles NMI exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void NMI_Handler(void)
{
}

/*!
 \brief      this function handles HardFault exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
 \brief      this function handles MemManage exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
 \brief      this function handles BusFault exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
 \brief      this function handles UsageFault exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
 \brief      this function handles SVC exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void SVC_Handler(void)
{
}

/*!
 \brief      this function handles DebugMon exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void DebugMon_Handler(void)
{
}

/*!
 \brief      this function handles PendSV exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void PendSV_Handler(void)
{
}

/*!
 \brief      this function handles SysTick exception
 \param[in]  none
 \param[out] none
 \retval     none
 */
void SysTick_Handler(void)
{
    delay_decrement();
}

/*!
    \brief      this function handles USBD interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBFS_IRQHandler(void)
{
    usbd_isr(&g_composite_hid_device);
}

/*!
    \brief      this function handles USBFS wakeup interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBFS_WKUP_IRQHandler(void)
{
    if (g_composite_hid_device.bp.low_power)
    {
        resume_mcu_clk();

#ifndef USE_IRC48M
        rcu_usbfs_clock_config(g_usbfs_prescaler);
#else
        /* enable IRC48M clock */
        rcu_osci_on(RCU_IRC48M);

        /* wait till IRC48M is ready */
        while(SUCCESS != rcu_osci_stab_wait(RCU_IRC48M)) {
        }

        rcu_ck48m_clock_config(RCU_CK48MSRC_IRC48M);
#endif /* USE_IRC48M */

        rcu_periph_clock_enable(RCU_USBFS);

        usb_clock_active(&g_composite_hid_device);
    }

    exti_interrupt_flag_clear(EXTI_18);
}

/**
  * @brief  This function handles TIMER2 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER1_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER1, TIMER_INT_UP) == SET)
    {
        // Clear update interrupt bit
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_UP);
        IncrememtDigitizerTimeStamp();
    }
}

/*!
    \brief      this function handles Timer2 update interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

/**
  * @brief  This function handles TIMER13 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER13_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER13, TIMER_INT_UP) == SET)
    {
        // Clear update interrupt bit
        timer_interrupt_flag_clear(TIMER13, TIMER_INT_UP);

        // When this timer has elapsed, re-start proxy mode for press and digitizer endpoints
        SetProxyMode(ePressDigiProxy);

        // Only need the timer to run a single time, we can disable once the interrupt has been generated
        timer_disable(PROXY_DELAY_TIMER);
    }
}

/**
  * @brief  This function handles TIMER14 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER14_IRQHandler(void)
{
    static uint8_t led_count = 0;

    if (timer_interrupt_flag_get(TIMER14, TIMER_INT_UP) == SET)
    {
        // Clear update interrupt bit
        timer_interrupt_flag_clear(TIMER14, TIMER_INT_UP);

        // Slow down the LEDs some more still
        if(led_count > 1)
        {
            // Toggle the LEDs
            gpio_bit_toggle(LED_AXIOM_GPIO_PORT, LED_AXIOM_GPIO_PIN);
            gpio_bit_toggle(LED_USB_GPIO_PORT, LED_USB_GPIO_PIN);
            led_count = 0;
        }
        else
        {
            led_count++;
        }
    }
}

/*!
    \brief      this function handles EXTI4_15_IRQ Handler.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI4_15_IRQHandler(void)
{
}

/**
 * @brief   Called when wanting to wake the host from sleep.
 */
void WakeHost(usb_core_driver *udev)
{
    // Send a remote wakeup to the host
    resume_mcu_clk();

#ifndef USE_IRC48M
    rcu_usbfs_clock_config(g_usbfs_prescaler);
#else
    /* enable IRC48M clock */
    rcu_osci_on(RCU_IRC48M);

    /* wait till IRC48M is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC48M))
    {
        continue;
    }

    rcu_ck48m_clock_config(RCU_CK48MSRC_IRC48M);
#endif /* USE_IRC48M */

    rcu_periph_clock_enable(RCU_USBFS);

    usb_clock_active(udev);

    usb_rwkup_set(udev);

    for (__IO uint16_t i = 0U; i < 1000U; i++)
    {
        for (__IO uint16_t i = 0U; i < 100U; i++)
            ;
    }

    usb_rwkup_reset(udev);

    udev->dev.cur_status = udev->dev.backup_status;

    udev->dev.pm.dev_remote_wakeup = 0U;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void resume_mcu_clk(void)
{
    /* enable HSE */
    rcu_osci_on(RCU_IRC8M);

    /* wait till HSE is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_IRC8MSTB)) {
    }

    /* enable PLL */
    rcu_osci_on(RCU_PLL_CK);

    /* wait till PLL is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_PLLSTB)) {
    }

    /* enable PLL */
    rcu_osci_on(RCU_PLL_CK);

    /* wait till PLL is ready */
    while(RESET == rcu_flag_get(RCU_FLAG_PLLSTB)) {
    }

    /* select PLL as system clock source */
    rcu_system_clock_source_config(RCU_CKSYSSRC_PLL);

    /* wait till PLL is used as system clock source */
    while(RCU_SCSS_PLL != rcu_system_clock_source_get()) {
    }
}
