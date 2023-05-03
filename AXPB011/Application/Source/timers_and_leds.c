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
#include "timers_and_leds.h"
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
uint8_t g_axiom_comms = NO_AXIOM_COMMS;
uint8_t g_usb_comms = NO_USB_COMMS;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define SYSTEMCLOCK_IN_MHZ  (SystemCoreClock / 1000000U)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/


/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void DigitizerTimerInit(void);
static void ProxyStartDelayTimerInit(void);
static void DetectCommsInactivity(uint8_t axiom_comms, uint8_t usb_comms);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
void TimerInit(void)
{
    // Initialise and enable the digitizer timestamp
    DigitizerTimerInit();
    timer_enable(DIGITIZER_TIMER);

    // Initialise proxy startup delay timer
    // DON'T enable yet, it will be enabled as a one shot
    ProxyStartDelayTimerInit();
}

/**
 * @brief   LED initialisation. Set low.
 */
void LEDsInit(void)
{
    gpio_mode_set(LED_USB_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_USB_GPIO_PIN);
    gpio_output_options_set(LED_USB_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED_USB_GPIO_PIN);
    gpio_bit_write(LED_USB_GPIO_PORT, LED_USB_GPIO_PIN, RESET);

    gpio_mode_set(LED_AXIOM_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_AXIOM_GPIO_PIN);
    gpio_output_options_set(LED_AXIOM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED_AXIOM_GPIO_PIN);
    gpio_bit_write(LED_AXIOM_GPIO_PORT, LED_AXIOM_GPIO_PIN, RESET);
}

void StartProxyDelayTimer(void)
{
    timer_enable(PROXY_DELAY_TIMER);
}

/**
 * @brief   Performs the LED control logic for communication and inactivity detection.
 * @param[in]   axiom_comms
 *  @arg    AXIOM_COMMS_DETECTED
 *  @arg    NO_AXIOM_COMMS_DETECTED
 * @param[in]   usb_comms
 *  @arg    USBM_COMMS_DETECTED
 *  @arg    NO_USB_COMMS_DETECTED
 */
void ControlLEDs(void)
{
    if (g_axiom_comms == AXIOM_COMMS_DETECTED)
    {
        // Reduce the frequency the LED is toggled.
        static uint8_t axiom_led_counter = 0;

        if(axiom_led_counter >= 5)
        {
            gpio_bit_write(LED_AXIOM_GPIO_PORT, LED_AXIOM_GPIO_PIN, RESET);
            axiom_led_counter = 0;
            g_axiom_comms = NO_AXIOM_COMMS;
        }
        else
        {
            gpio_bit_write(LED_AXIOM_GPIO_PORT, LED_AXIOM_GPIO_PIN, SET);
            axiom_led_counter++;
        }
    }

    if (g_usb_comms == USB_COMMS_DETECTED)
    {
        // Reduce the frequency the LED is toggled.
        static uint8_t usb_led_counter = 0;

        if(usb_led_counter >= 5)
        {
            gpio_bit_write(LED_USB_GPIO_PORT, LED_USB_GPIO_PIN, RESET);
            usb_led_counter = 0;
            g_usb_comms = NO_USB_COMMS;
        }
        else
        {
            gpio_bit_write(LED_USB_GPIO_PORT, LED_USB_GPIO_PIN, SET);
            usb_led_counter++;
        }
    }

    DetectCommsInactivity(g_axiom_comms, g_usb_comms);
}

/**
 * @brief   Set a flag indicating there has been some USB activity.
 */
void NotifyUSBComms(void)
{
    g_usb_comms = USB_COMMS_DETECTED;
}

/**
 * @brief   Set a flag indicating there has been some axiom (I2C or SPI) activity.
 */
void NotifyAxiomComms(void)
{
    g_axiom_comms = AXIOM_COMMS_DETECTED;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void DigitizerTimerInit(void)
{
    timer_parameter_struct timer_initpara;

    // Enable the peripherals clock
    rcu_periph_clock_enable(RCU_TIMER1);

    // Deinit a TIMER
    timer_deinit(DIGITIZER_TIMER);

    // Initialize TIMER init parameter struct
    timer_struct_para_init(&timer_initpara);

    // TIMER2 configuration
    timer_initpara.prescaler         = SYSTEMCLOCK_IN_MHZ - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 100 - 1; // 100us period
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(DIGITIZER_TIMER, &timer_initpara);

    // Clear interrupt bit
    timer_interrupt_flag_clear(DIGITIZER_TIMER, TIMER_INT_FLAG_UP);

    // Enable the TIMER interrupt
    timer_interrupt_enable(DIGITIZER_TIMER, TIMER_INT_UP);

    // Enable interrupts
    nvic_irq_enable(TIMER1_IRQn, 0, 0);
}

static void ProxyStartDelayTimerInit(void)
{
    timer_parameter_struct timer_initpara;

    // Enable the peripherals clock
    rcu_periph_clock_enable(RCU_TIMER13);

    // Deinit a TIMER
    timer_deinit(PROXY_DELAY_TIMER);

    // Initialize TIMER init parameter struct
    timer_struct_para_init(&timer_initpara);

    // TIMER2 configuration
    timer_initpara.prescaler         = SYSTEMCLOCK_IN_MHZ - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000000 - 1; // ~1 second period
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(PROXY_DELAY_TIMER, &timer_initpara);

    // Clear interrupt bit
    timer_interrupt_flag_clear(PROXY_DELAY_TIMER, TIMER_INT_FLAG_UP);

    // Enable the TIMER interrupt
    timer_interrupt_enable(PROXY_DELAY_TIMER, TIMER_INT_UP);

    // Enable interrupts
    nvic_irq_enable(TIMER13_IRQn, 0, 0);
}

static void DetectCommsInactivity(uint8_t axiom_comms, uint8_t usb_comms)
{
    static uint32_t heartbeat_axiom_count = 0;
    static uint32_t heartbeat_usb_count = 0;
    static uint32_t axiom_activity_count = 0;
    static uint32_t usb_activity_count = 0;

    // Check for axiom comms inactivity
    if(axiom_comms == NO_AXIOM_COMMS)
    {
        if(axiom_activity_count >= 10000)
        {
            // Once no activity has been detected, start a heartbeat pulse (~1 second)
            if(heartbeat_axiom_count >= 29000)
            {
                gpio_bit_write(LED_AXIOM_GPIO_PORT, LED_AXIOM_GPIO_PIN, RESET);
//                delay_1ms(50);
                heartbeat_axiom_count = 0;
            }
            else
            {
                gpio_bit_write(LED_AXIOM_GPIO_PORT, LED_AXIOM_GPIO_PIN, SET);
                heartbeat_axiom_count++;
            }
        }
        else
        {
            axiom_activity_count++;
        }
    }
    else
    {
        heartbeat_axiom_count = 0;
        axiom_activity_count = 0;
    }

    if(usb_comms == NO_USB_COMMS)
    {
        if(usb_activity_count >= 10000)
        {
            // Once no activity has been detected, start a heartbeat pulse (~1 second)
            if(heartbeat_usb_count >= 29000)
            {
                gpio_bit_write(LED_USB_GPIO_PORT, LED_USB_GPIO_PIN, RESET);
//                delay_1ms(50);
                heartbeat_usb_count = 0;
            }
            else
            {
                gpio_bit_write(LED_USB_GPIO_PORT, LED_USB_GPIO_PIN, SET);
                heartbeat_usb_count++;
            }
        }
        else
        {
            usb_activity_count++;
        }
    }
    else
    {
        heartbeat_usb_count = 0;
        usb_activity_count = 0;
    }
}
