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
#include "drv_usb_hw.h"

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
__IO uint32_t   g_delay_time        = 0U;
__IO uint16_t   g_timer_prescaler   = 5U;
uint32_t        g_usbfs_prescaler   = 0U;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define TIM_MSEC_DELAY                          0x01U
#define TIM_USEC_DELAY                          0x02U

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void hw_time_set(uint8_t unit);
static void hw_delay(uint32_t ntime, uint8_t unit);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/*!
    \brief      configure USB clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_rcu_config(void)
{
    uint32_t system_clock = rcu_clock_freq_get(CK_SYS);

    if(48000000U == system_clock) {
        g_usbfs_prescaler = RCU_USBFS_CKPLL_DIV1;
        g_timer_prescaler = 3U;
    } else if(72000000U == system_clock) {
        g_usbfs_prescaler = RCU_USBFS_CKPLL_DIV1_5;
        g_timer_prescaler = 5U;
    } else if(96000000U == system_clock) {
        g_usbfs_prescaler = RCU_USBFS_CKPLL_DIV2;
        g_timer_prescaler = 7U;
    } else {
        /*  reserved  */
    }

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
}

/*!
    \brief      configure USB interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_intr_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable((uint8_t)USBFS_IRQn, 2U, 0U);

    /* enable the power module clock */
    rcu_periph_clock_enable(RCU_PMU);

    /* USB wakeup EXTI line configuration */
    exti_interrupt_flag_clear(EXTI_18);
    exti_init(EXTI_18, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_18);

    nvic_irq_enable((uint8_t)USBFS_WKUP_IRQn, 1U, 0U);
}

/*!
    \brief      initializes delay unit using Timer2
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_init(void)
{
    /* set the vector table base address at 0x08000000 */
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x00U);

    /* configure the priority group to 2 bits */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

    /* enable the TIM2 global interrupt */
    nvic_irq_enable((uint8_t)TIMER2_IRQn, 1U, 0U);

    rcu_periph_clock_enable(RCU_TIMER2);
}

/*!
    \brief      delay in micro seconds
    \param[in]  usec: value of delay required in micro seconds
    \param[out] none
    \retval     none
*/
void usb_udelay(const uint32_t usec)
{
    hw_delay(usec, TIM_USEC_DELAY);
}

/*!
    \brief      delay in milli seconds
    \param[in]  msec: value of delay required in milli seconds
    \param[out] none
    \retval     none
*/
void usb_mdelay(const uint32_t msec)
{
    hw_delay(msec, TIM_MSEC_DELAY);
}

/*!
    \brief      time base IRQ
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_irq(void)
{
    if(RESET != timer_interrupt_flag_get(TIMER2, TIMER_INT_UP)) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_UP);

        if(g_delay_time > 0x00U) {
            g_delay_time--;
        } else {
            timer_disable(TIMER2);
        }
    }
}

#ifdef USE_IRC48M
/*!
    \brief      configure the CTC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ctc_config(void)
{
    /* configure CTC reference signal source prescaler */
    ctc_refsource_prescaler_config(CTC_REFSOURCE_PSC_OFF);
    /* select reference signal source */
    ctc_refsource_signal_select(CTC_REFSOURCE_USBSOF);
    /* select reference signal source polarity */
    ctc_refsource_polarity_config(CTC_REFSOURCE_POLARITY_RISING);
    /* configure hardware automatically trim mode */
    ctc_hardware_trim_mode_config(CTC_HARDWARE_TRIM_MODE_ENABLE);

    /* configure CTC counter reload value, Fclock/Fref-1 */
    ctc_counter_reload_value_config(0xBB7FU);
    /* configure clock trim base limit value, Fclock/Fref*0.0012/2 */
    ctc_clock_limit_value_config(0x1DU);

    /* CTC counter enable */
    ctc_counter_enable();
}
#endif

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
/*!
    \brief      configures TIMER2 for delay routine based on TIMER2
    \param[in]  unit: msec /usec
    \param[out] none
    \retval     none
*/
static void hw_time_set(uint8_t unit)
{
    timer_parameter_struct  timer_basestructure;

    timer_disable(TIMER2);
    timer_interrupt_disable(TIMER2, TIMER_INT_UP);

    if(TIM_USEC_DELAY == unit) {
        timer_basestructure.period = 11U;
    } else if(TIM_MSEC_DELAY == unit) {
        timer_basestructure.period = 11999U;
    } else {
        /* no operation */
    }

    timer_basestructure.prescaler         = g_timer_prescaler;
    timer_basestructure.alignedmode       = TIMER_COUNTER_EDGE;
    timer_basestructure.counterdirection  = TIMER_COUNTER_UP;
    timer_basestructure.clockdivision     = TIMER_CKDIV_DIV1;
    timer_basestructure.repetitioncounter = 0U;

    timer_init(TIMER2, &timer_basestructure);

    timer_interrupt_flag_clear(TIMER2, TIMER_INT_UP);

    timer_auto_reload_shadow_enable(TIMER2);

    /* TIMER2 interrupt enable */
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);

    /* TIMER2 enable counter */
    timer_enable(TIMER2);
}

/*!
    \brief      delay routine based on TIM2
    \param[in]  nTime: delay Time
    \param[in]  unit: delay Time unit = mili sec / micro sec
    \param[out] none
    \retval     none
*/
static void hw_delay(uint32_t ntime, uint8_t unit)
{
    g_delay_time = ntime;

    hw_time_set(unit);

    while(0U != g_delay_time) {
    }

    timer_disable(TIMER2);
}
