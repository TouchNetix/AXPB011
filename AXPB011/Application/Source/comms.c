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
#include "comms.h"
#include "i2c.h"
#include "spi.h"
#include "systick.h"
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


/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
en_comms g_comms_select = eI2C;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define AXIOM_COMMS_SET_GPIO_PORT   (GPIOA)
#define AXIOM_COMMS_SET_PIN         (GPIO_PIN_6)
#define COMMS_SELECT_GPIO_PORT      (GPIOA)
#define COMMS_SELECT_PIN            (GPIO_PIN_1)

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
 * @brief   Configures the communication peripheral with axiom.
 * @return  None.
 */
void CommsInit(void)
{
    g_comms_select = GetCommsMode();


    if (g_comms_select == eI2C)
    {
        SetAxiomCommsMode(I2C_MODE);
        I2C_Init();
    }
    else if (g_comms_select == eSPI)
    {
        SetAxiomCommsMode(SPI_MODE);
        SPI_Init();
    }
    else
    {
        // Do nothing - Couldn't determine comms mode
    }
}

/**
 * @brief   Resets all axiom communication peripherals.
 * @return  None.
 */
void CommsDeInit(void)
{
    // Reset both I2C and SPI to be safe
    I2C_DeInit();
    SPI_DeInit();
}

/**
 * @brief   Changes the comms mode of axiom by controlling the nSLVI2C/MISO line.
 * @param[in]   comms_mode  Either I2C_MODE or SPI_MODE.
 * @return  None.
 */
void SetAxiomCommsMode(uint8_t comms_mode)
{
    static uint8_t pin_configured = 0;

    if (pin_configured == 0)
    {
        gpio_mode_set(AXIOM_COMMS_SET_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AXIOM_COMMS_SET_PIN);
        gpio_output_options_set(AXIOM_COMMS_SET_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, AXIOM_COMMS_SET_PIN);
        pin_configured = 1;
    }

    if (comms_mode == I2C_MODE)
    {
        gpio_bit_write(AXIOM_COMMS_SET_GPIO_PORT, AXIOM_COMMS_SET_PIN, RESET);
    }
    else
    {
        // SPI mode
        gpio_bit_write(AXIOM_COMMS_SET_GPIO_PORT, AXIOM_COMMS_SET_PIN, SET);
    }

    ResetAxiom();
}

/**
 * @brief   Send data to the connected aXiom device via the configured interface.
 * @param[in]   pagenum Page address in axiom to be written to.
 * @param[in]   offset  Offset into page to be written to.
 * @param[in]   pbuf    Pointer to the data to be sent.
 * @param[in]   length  Number of bytes to write.
 * @return  Status of comms.
 * @retval  AXIOMCOMMS_READWRITE_OK      Bridge successfully read from aXiom.
 * @retval  AXIOMCOMMS_COMMS_FAILED      Bridge could not communicate with aXiom (either through invalid setup or other error).
 * @retval  AXIOMCOMMS_DEVICE_TIMEOUT    aXiom did not respond to the bridge in time (I2C only).
 * @retval  AXIOMCOMMS_WRITE_OK_NOREAD   Bridge successfully wrote to aXiom.
 */
uint32_t WriteAxiom(uint16_t addr, uint8_t *pbuf, uint32_t length)
{
    uint32_t status = AXIOMCOMMS_WRITE_OK_NOREAD;
    uint8_t pagenum = (uint8_t)((addr & 0xFF00U) >> 8U);
    uint8_t offset = (uint8_t)(addr & 0x00FFU);

    switch (g_comms_select)
    {
        case eI2C:
        {
            uint32_t i2c_retval = I2C_WriteAxiom(pagenum, offset, pbuf, length);

            switch(i2c_retval)
            {
                case I2CSTATUS_WRITE_OK_NOREAD:
                {
                    status = AXIOMCOMMS_WRITE_OK_NOREAD;
                    break;
                }

                case I2CSTATUS_TIMEOUT:
                {
                    status = AXIOMCOMMS_DEVICE_TIMEOUT;
                    break;
                }

                case I2CSTATUS_COMMS_FAILED:
                case I2CSTATUS_READWRITEOK:
                default:
                {
                    // Either comms has actually failed, or somehow we got here through incorrect setup
                    status = AXIOMCOMMS_COMMS_FAILED;
                    break;
                }
            }

            NotifyAxiomComms();
            break;
        }

        case eSPI:
        {
            uint32_t spi_retval = SPI_WriteAxiom(pagenum, offset, pbuf, length);

            switch(spi_retval)
            {
                case SPISTATUS_WRITE_OK_NOREAD:
                {
                    status = AXIOMCOMMS_WRITE_OK_NOREAD;
                    break;
                }

                case SPISTATUS_TIMEOUT:
                {
                    status = AXIOMCOMMS_DEVICE_TIMEOUT;
                    break;
                }

                case SPISTATUS_COMMS_FAILED:
                case SPISTATUS_READWRITEOK:
                default:
                {
                    // Either comms has actually failed, or somehow we got here through incorrect setup
                    status = AXIOMCOMMS_COMMS_FAILED;
                    break;
                }
            }

            NotifyAxiomComms();
            break;
        }

        case eNoCommsSelected:
        default:
        {
            // Do nothing - no comms interface selected
            status = AXIOMCOMMS_COMMS_FAILED;
            break;
        }
    }

    return status;
}

/**
 * @brief   Receive data from the connected aXiom device via I2C.
 * @param[in]   pagenum Page address in axiom to be written to.
 * @param[in]   offset  Offset into page to be written to.
 * @param[out]  pbuf    Pointer to buffer where read data will be stored.
 * @param[in]   length  Number of bytes to read.
 * @return  Status of comms.
 * @retval  AXIOMCOMMS_READWRITE_OK      Bridge successfully read from aXiom.
 * @retval  AXIOMCOMMS_COMMS_FAILED      Bridge could not communicate with aXiom (either through invalid setup or other error).
 * @retval  AXIOMCOMMS_DEVICE_TIMEOUT    aXiom did not respond to the bridge in time (I2C only).
 * @retval  AXIOMCOMMS_WRITE_OK_NOREAD   Bridge successfully wrote to aXiom.
 */
uint32_t ReadAxiom(uint16_t addr, uint8_t *pbuf, uint32_t length)
{
    uint32_t status = AXIOMCOMMS_READWRITE_OK;
    uint8_t pagenum = (addr & 0xFF00) >> 8;
    uint8_t offset = addr & 0x00FF;

    switch (g_comms_select)
    {
        case eI2C:
        {
            uint32_t i2c_retval = I2C_ReadAxiom(pagenum, offset, pbuf, length);
            switch(i2c_retval)
            {
                case I2CSTATUS_READWRITEOK:
                {
                    status = AXIOMCOMMS_READWRITE_OK;
                    break;
                }

                case I2CSTATUS_TIMEOUT:
                {
                    status = AXIOMCOMMS_DEVICE_TIMEOUT;
                    break;
                }

                case I2CSTATUS_COMMS_FAILED:
                case I2CSTATUS_WRITE_OK_NOREAD:
                default:
                {
                    // Either comms has actually failed, or somehow we got here through incorrect setup
                    status = AXIOMCOMMS_COMMS_FAILED;
                    break;
                }
            }

            NotifyAxiomComms();
            break;
        }

        case eSPI:
        {
            uint32_t spi_retval = SPI_ReadAxiom(pagenum, offset, pbuf, length);
            switch(spi_retval)
            {
                case SPISTATUS_READWRITEOK:
                {
                    status = AXIOMCOMMS_READWRITE_OK;
                    break;
                }

                case SPISTATUS_TIMEOUT:
                {
                    status = AXIOMCOMMS_DEVICE_TIMEOUT;
                    break;
                }

                case SPISTATUS_COMMS_FAILED:
                case SPISTATUS_WRITE_OK_NOREAD:
                default:
                {
                    // Either comms has actually failed, or somehow we got here through incorrect setup
                    status = AXIOMCOMMS_COMMS_FAILED;
                    break;
                }
            }

            NotifyAxiomComms();
            break;
        }

        case eNoCommsSelected:
        default:
        {
            // Do nothing - no comms interface selected
            break;
        }
    }

    return status;
}

/**
 * @brief   Returns the state of the comms select pin.
 * @retval  eI2C or eSPI.
 */
en_comms GetCommsMode(void)
{
    en_comms mode = eNoCommsSelected;
    static uint8_t comms_pin_configured = 0;

    if (comms_pin_configured == 0)
    {
        gpio_mode_set(COMMS_SELECT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, COMMS_SELECT_PIN);
        comms_pin_configured = 1;
    }

    // Read comms select pin
    if (gpio_input_bit_get(COMMS_SELECT_GPIO_PORT, COMMS_SELECT_PIN) == RESET)
    {
        mode = eI2C;
    }
    else
    {
        mode = eSPI;
    }

    return mode;
}

/**
 * @brief   Fetches the address of axiom.
 * @retval  7-bit i2c address of axiom. Returns 0 if unknown.
 */
uint32_t GetAxiomI2CAddress(void)
{
    return I2C_GetI2CAddress();
}

/**
 * @brief   Resets aXiom using the NRESET line.
 * @return  None.
 */
void ResetAxiom(void)
{
    gpio_bit_write(NRESET_GPIO_PORT, NRESET_PIN, RESET);
    delay_1ms(1);
    gpio_bit_write(NRESET_GPIO_PORT, NRESET_PIN, SET);
    delay_1ms(500); // Give aXiom time to come up again
}


/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
