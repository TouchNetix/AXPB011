/*!
 @file  i2c.c
 @brief I2C driver file.

 @version 03/08/2022 - V01.00. First release. Author: James Cameron.
 */

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
#include <string.h>
#include "i2c.h"
#include "systick.h"


/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/
#define I2C_PERIPH  I2C0

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
static uint32_t g_axiom_i2c_addr = 0;

/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/
#define I2C_SCL_GPIO_PORT   (GPIOB)
#define I2C_SCL_PIN         (GPIO_PIN_6)
#define I2C_SDA_GPIO_PORT   (GPIOB)
#define I2C_SDA_PIN         (GPIO_PIN_7)

#define I2C_FASTMODE            (400000U)
#define MAX_ADDRSEND_ATTEMPTS   (4U)
#define EEPROM_ADDRESS          (0xA0 >> 1U)
#define NUM_CMD_BYTES           (4U)
#define USB_MAX_DATA_SIZE       (58U)   // max usb packet size (64) minus 2 bridge header bytes and 4 axiom command bytes
#define I2C_TXRX_BUFFER_SIZE    (NUM_CMD_BYTES + USB_MAX_DATA_SIZE)
#define WRITE                   (0x00U)
#define READ                    (0x80U)

/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static void     I2C_Transmit(uint8_t *pdata, uint32_t length);
static void     I2C_Receive(uint8_t *pbuf, uint32_t length);
static void     I2C_PinConfig(void);
static void     SetI2CAddress(uint32_t addr);
static uint32_t FindI2CAddress(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief   Configure the I2C peripheral.
 */
void I2C_Init(void)
{
    // Enable peripheral clock
    rcu_periph_clock_enable(RCU_I2C0);

    // Configure the pins used for I2C
    I2C_PinConfig();

    // Configure the peripheral settings
    i2c_clock_config(I2C_PERIPH, I2C_FASTMODE, I2C_DTCY_2);
    i2c_enable(I2C_PERIPH);
    i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);

    SetI2CAddress(FindI2CAddress());
}

/**
 * @brief   De-initialise the I2C peripheral.
 */
void I2C_DeInit(void)
{
    i2c_disable(I2C_PERIPH);

    // Reset the pins
    gpio_deinit(I2C_SCL_GPIO_PORT);
    gpio_deinit(I2C_SDA_GPIO_PORT);

    // Clear the peripheral settings
    i2c_deinit(I2C_PERIPH);

    // Last step - disable the peripheral clock
    rcu_periph_clock_disable(RCU_I2C0);
}

/**
 * @brief   Send data to the connected aXiom device via I2C.
 * @param[in]   pagenum Page address in axiom to be written to.
 * @param[in]   offset  Offset into page to be written to.
 * @param[in]   pbuf    Pointer to the data to be sent.
 * @param[in]   length  Number of bytes to write.
 */
void I2C_WriteAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length)
{
    // Wait until I2C bus is idle
    while (i2c_flag_get(I2C_PERIPH, I2C_FLAG_I2CBSY))
    {
        continue;
    }

    uint8_t txbuffer[I2C_TXRX_BUFFER_SIZE] = {0};
    txbuffer[0] = offset;
    txbuffer[1] = pagenum;
    txbuffer[2] = length & 0x00FFU;
    txbuffer[3] = ((length & 0xFF00U) >> 8U)| WRITE;
    memcpy(&txbuffer[4], &pbuf[0], length);

    // Send a start condition to I2C bus
    i2c_start_on_bus(I2C_PERIPH);
    while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_SBSEND))
    {
        continue;
    }

    I2C_Transmit(txbuffer, (length + NUM_CMD_BYTES));

    // Send a stop condition to I2C bus and wait for stop condition to be generated
    i2c_stop_on_bus(I2C_PERIPH);
    while (I2C_CTL0(I2C_PERIPH) & 0x0200)
    {
        continue;
    }
}

/**
 * @brief   Receive data from the connected aXiom device via I2C.
 * @details This function performs a repeated start between writing the 4 command bytes and receiving the data.
 * @param[in]   pagenum Page address in axiom to be read from.
 * @param[in]   offset  Offset into page to be read from.
 * @param[out]  pbuf    Pointer to buffer where read data will be stored.
 * @param[in]   length  Number of bytes to read.
 */
void I2C_ReadAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length)
{
    // Wait until I2C bus is idle
    while (i2c_flag_get(I2C_PERIPH, I2C_FLAG_I2CBSY))
    {
        continue;
    }

    uint8_t txbuffer[NUM_CMD_BYTES] = {0};
    txbuffer[0] = offset;
    txbuffer[1] = pagenum;
    txbuffer[2] = length & 0x00FFU;
    txbuffer[3] = ((length & 0xFF00U) >> 8U)| READ;

    // Send a start condition to I2C bus
    i2c_start_on_bus(I2C_PERIPH);
    while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_SBSEND))
    {
        continue;
    }

    I2C_Transmit(txbuffer, NUM_CMD_BYTES);

    // Repeated start and stop condition generated in I2C_Receive()
    I2C_Receive(pbuf, length);
}

/**
 * @brief   Returns the locally stored axiom i2c address.
 */
uint32_t I2C_GetI2CAddress(void)
{
    return g_axiom_i2c_addr;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
/**
 * @brief   Enacts the data write to the I2C bus.
 * @param[in]   pbuf    Pointer to the data to be sent.
 * @param[in]   length  Number of bytes to write.
 */
static void I2C_Transmit(uint8_t *pbuf, uint32_t length)
{
    // Send slave address to I2C bus
    i2c_master_addressing(I2C_PERIPH, I2C_GetI2CAddress(), I2C_TRANSMITTER);
    while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_ADDSEND))
    {
        continue;
    }

    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);

    // Wait until the transmit data buffer is empty
    while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_TBE))
    {
        continue;
    }

    for (uint32_t i = 0; i < length; i++)
    {
        i2c_data_transmit(I2C_PERIPH, pbuf[i]);

        // Wait until the TBE bit is set
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_TBE))
        {
            continue;
        }
    }
}

/**
 * @brief   Enacts the data read from the I2C bus.
 * @param[out]  pbuf    Pointer to buffer where read data will be stored.
 * @param[in]   length  Number of bytes to read.
 */
static void I2C_Receive(uint8_t *pbuf, uint32_t length)
{
    if (length == 0)
    {
        // Do nothing
    }
    else if (length == 1)
    {
        // Repeated start
        i2c_start_on_bus(I2C_PERIPH);
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_SBSEND))
        {
            continue;
        }

        // Send slave address to I2C bus
        i2c_master_addressing(I2C_PERIPH, I2C_GetI2CAddress(), I2C_RECEIVER);
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_ADDSEND))
        {
            continue;
        }

        // Disable acknowledge before clearing ADDSEND flag
        i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);

        // Send a stop condition to I2C bus and wait for stop condition to be generated
        i2c_stop_on_bus(I2C_PERIPH);
        while (I2C_CTL0(I2C_PERIPH) & 0x0200)
        {
            continue;
        }

        // Wait until the RBNE bit is set
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_RBNE))
        {
            continue;
        }

        // Read data from I2C_DATA
        pbuf[0] = i2c_data_receive(I2C_PERIPH);

        // Enable acknowledge
        i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
    }
    else if (length == 2)
    {
        // Configure ACKEN to control whether send ACK/NACK for the next byte, rather than the current one
        i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_NEXT);

        // Repeated start
        i2c_start_on_bus(I2C_PERIPH);
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_SBSEND))
        {
            continue;
        }

        // Send slave address to I2C bus
        i2c_master_addressing(I2C_PERIPH, I2C_GetI2CAddress(), I2C_RECEIVER);
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_ADDSEND))
        {
            continue;
        }

        // Disable acknowledge before clearing ADDSEND flag
        i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);

        // Wait for BTC to be set - indicates byte to receive but rx buffer is full
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_BTC))
        {
            continue;
        }

        // Send a stop condition to I2C bus and wait for stop condition to be generated
        i2c_stop_on_bus(I2C_PERIPH);
        while (I2C_CTL0(I2C_PERIPH) & 0x0200)
        {
            continue;
        }

        // Read out the 2 data bytes
        for (uint32_t i = 0; i < length; i++)
        {
            // Wait until the RBNE bit is set
            while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_RBNE))
            {
                continue;
            }

            // Read a byte from I2C_DATA
            pbuf[i] = i2c_data_receive(I2C_PERIPH);
        }
    }
    else
    {
        // Repeated start
        i2c_start_on_bus(I2C_PERIPH);
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_SBSEND))
        {
            continue;
        }

        // Send slave address to I2C bus
        i2c_master_addressing(I2C_PERIPH, I2C_GetI2CAddress(), I2C_RECEIVER);
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_ADDSEND))
        {
            continue;
        }

        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);

        for (uint32_t i = 0; i < length; i++)
        {
            if (i == (length - 3))
            {
                // Wait until the (length - 2) data byte is received into the shift register
                while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_BTC))
                {
                    continue;
                }

                // Disable acknowledge
                i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
            }

            // Wait until the RBNE bit is set
            while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_RBNE))
            {
                continue;
            }

            // Read a byte from I2C_DATA
            pbuf[i] = i2c_data_receive(I2C_PERIPH);
        }

        // Enable acknowledge
        i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);

        // Send a stop condition to I2C bus and wait for stop condition to be generated
        i2c_stop_on_bus(I2C_PERIPH);
        while (I2C_CTL0(I2C_PERIPH) & 0x0200)
        {
            continue;
        }
    }
}

/**
 * @brief   Initialises the GPIOs required for the I2C peripheral.
 * @details PB6 = SCL
 *          PB7 = SDA
 */
static void I2C_PinConfig(void)
{
    // connect PB6 to I2C0_SCL
    gpio_af_set(I2C_SCL_GPIO_PORT, GPIO_AF_1, I2C_SCL_PIN);

    // connect PB7 to I2C0_SDA
    gpio_af_set(I2C_SDA_GPIO_PORT, GPIO_AF_1, I2C_SDA_PIN);

    // configure GPIO pins of I2C0
    gpio_mode_set(I2C_SCL_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_PIN);
    gpio_mode_set(I2C_SDA_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SDA_PIN);
    gpio_output_options_set(I2C_SCL_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);
    gpio_output_options_set(I2C_SDA_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);
}

/**
 * @brief   Updates the locally stored axiom i2c address.
 * @param[in]   addr    I2C address of axiom.
 */
static void SetI2CAddress(uint32_t addr)
{
    g_axiom_i2c_addr = addr;
}

/**
 * @brief   Scans the 7-bit i2c address range looking for a device to connect to.
 * @details aXiom typically has address 0x66 or 0x67. Addresses 0x00 (general call)
 *          and 0x50 (EVAL board EEPROM) are skipped.
 * @return  Address of connected device, in 7-bit format, left shifted by 1.
 */
static uint32_t FindI2CAddress(void)
{
    uint8_t     buffer[1] = {0};
    uint8_t     timeout_flag = 0;
    uint32_t    temp_addr;

    // Find address of connected device - 0x00 is general call address so skip it
    for (temp_addr = 0x01; temp_addr < 0x7E; temp_addr++)
    {
        if (temp_addr == EEPROM_ADDRESS)
        {
            // 0xA0 is the I2C EEPROM address (EVAL board only)
            continue;
        }

        // Configure with the next i2c address
        i2c_mode_addr_config(I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, (temp_addr << 1U));

        // Wait until I2C bus is idle
        while (i2c_flag_get(I2C_PERIPH, I2C_FLAG_I2CBSY))
        {
            continue;
        }

        // Send a start condition to I2C bus
        i2c_start_on_bus(I2C_PERIPH);
        while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_SBSEND))
        {
            continue;
        }

        // Send the current address to the device
        i2c_master_addressing(I2C_PERIPH, (temp_addr << 1U), I2C_TRANSMITTER);

        // If the address is correct the ADDSEND (address send) flag will be raised
        timeout_flag = 0;
        for (uint32_t timeout = 0; timeout <= MAX_ADDRSEND_ATTEMPTS; timeout++)
        {
            if (i2c_flag_get(I2C_PERIPH, I2C_FLAG_ADDSEND))
            {
                // Pass
                i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);
                timeout_flag = 0;
                break;
            }

            if (timeout == MAX_ADDRSEND_ATTEMPTS)
            {
                // Fail
                timeout_flag = 1;

                // Need to clear this flag as there will have been an ACK error
                i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
                break;
            }

            // Gives the ADDSEND flag time to assert
            delay_1ms(1);
        }

        if (timeout_flag == 0)
        {
            // Transmit an empty byte
            i2c_data_transmit(I2C_PERIPH, buffer[0]);
            while (!i2c_flag_get(I2C0, I2C_FLAG_TBE))
            {
                continue;
            }

            // Check for ACK status
            if (i2c_flag_get(I2C_PERIPH, I2C_FLAG_AERR))
            {
                // Fail
                i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
            }
            else
            {
                // Pass - Device has been found
                // Send a stop and exit loop
                i2c_stop_on_bus(I2C_PERIPH);
                while (I2C_CTL0(I2C_PERIPH) & 0x0200)
                {
                    continue;
                }

                break;
            }
       }

        // Send a stop condition to I2C bus and wait for stop condition to be generated
        i2c_stop_on_bus(I2C_PERIPH);
        while (I2C_CTL0(I2C_PERIPH) & 0x0200)
        {
            continue;
        }
    }

    if (temp_addr == 0x7E)
    {
        temp_addr = 0;
    }

    // Returns 0 if the device could not be found;
    return (temp_addr << 1U);
}
