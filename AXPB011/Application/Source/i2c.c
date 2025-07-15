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


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/
typedef enum
{
    eI2C_OK         = 0,
    eI2C_FAIL       = 1,
    eI2C_TIMEOUT    = 2,
} en_i2cStatus;


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
#define I2C_PERIPH      (I2C0)

#define I2C_MAX_TIMEOUT_COUNT   (5000U) /*25ms*/
#define I2C_SLEEP_US            (5U)

#define STATUS_OK       (0x00U)
#define STATUS_ERROR    (0x01U)

/*******************************************************************************
 * File Scope Macros
 ******************************************************************************/
#define I2C_SCL_GPIO_PORT   (GPIOB)
#define I2C_SCL_PIN         (GPIO_PIN_6)
#define I2C_SDA_GPIO_PORT   (GPIOB)
#define I2C_SDA_PIN         (GPIO_PIN_7)

#define I2C_FASTMODE                (400000U)
#define MAX_ADDRSEND_ATTEMPTS       (10U)
#define MAX_ADDR_SEARCH_ATTEMPTS    (250U)
#define NO_AXIOM_FOUND_ADDR         (0)
#define NUM_CMD_BYTES               (4U)
#define USB_MAX_DATA_SIZE           (58U)   // max usb packet size (64) minus 2 bridge header bytes and 4 axiom command bytes
#define I2C_TXRX_BUFFER_SIZE        (NUM_CMD_BYTES + USB_MAX_DATA_SIZE)
#define WRITE                       (0x00U)
#define READ                        (0x80U)
#define ADDSEND_DONTCLEAR           (0)
#define ADDSEND_CLEAR               (1U)

/*******************************************************************************
 * File Scope Inline Functions
 ******************************************************************************/


/*******************************************************************************
 * File Scope Function Prototypes
 ******************************************************************************/
static en_i2cStatus I2C_WaitForIdleBus(void);
static en_i2cStatus I2C_PopulateCommandBytes(uint8_t *pBuffer, uint8_t pagenum, uint8_t offset, uint32_t length, uint8_t read_write);
static en_i2cStatus I2C_SendStartToBus(void);
static en_i2cStatus I2C_SendStopToBus(void);
static en_i2cStatus I2C_Transmit(uint8_t *pdata, uint32_t length);
static en_i2cStatus I2C_Receive(uint8_t *pbuf, uint32_t length);
static en_i2cStatus I2C_SendAddress(uint32_t read_write, uint8_t clear_addsend);
static en_i2cStatus I2C_TransmitByte(uint8_t data);
static en_i2cStatus I2C_WaitForRBNEFlag(void);
static en_i2cStatus I2C_WaitForBTCFlag(void);
static void         I2C_PinConfig(void);
static void         SetI2CAddress(uint32_t addr);
static uint32_t     FindI2CAddress(void);

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
 * @return  Status of I2C comms.
 * @retval  I2CSTATUS_WRITE_OK_NOREAD   Bridge successfully wrote to aXiom.
 * @retval  I2CSTATUS_COMMS_FAILED      Bridge could not communicate with aXiom (either through invalid setup or other error).
 * @retval  I2CSTATUS_DEVICE_TIMEOUT    aXiom did not respond to the bridge in time.
 */
uint32_t I2C_WriteAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length)
{
    // Wait until I2C bus is idle
    if (I2C_WaitForIdleBus() == eI2C_TIMEOUT)
    {
        return I2CSTATUS_TIMEOUT;
    }

    // Prepare the transmit buffer
    uint8_t txbuffer[I2C_TXRX_BUFFER_SIZE] = {0};
    if (I2C_PopulateCommandBytes(txbuffer, pagenum, offset, length, WRITE) == eI2C_FAIL)
    {
        // Pointer to buffer was NULL
        return I2CSTATUS_COMMS_FAILED;
    }

    // Copy over the payload to be written to aXiom
    memcpy(&txbuffer[4], &pbuf[0], length);

    // Send a start condition to I2C bus
    // Timeout in case another device on the I2C bus is hogging the bandwidth
    if (I2C_SendStartToBus() != eI2C_OK)
    {
        // Comms failed, send a stop condition and exit here
        I2C_SendStopToBus();
        return I2CSTATUS_TIMEOUT;
    }

    // Write to aXiom
    en_i2cStatus i2c_status = I2C_Transmit(txbuffer, (length + NUM_CMD_BYTES));

    // Send a stop condition to I2C bus regardless of comms status (releases bus for other controllers)
    I2C_SendStopToBus();

    if (i2c_status == eI2C_TIMEOUT)
    {
        return I2CSTATUS_TIMEOUT;
    }
    else
    {
        return I2CSTATUS_WRITE_OK_NOREAD;
    }
}

/**
 * @brief   Receive data from the connected aXiom device via I2C.
 * @details This function performs a repeated start between writing the 4 command bytes and receiving the data.
 * @param[in]   pagenum Page address in axiom to be read from.
 * @param[in]   offset  Offset into page to be read from.
 * @param[out]  pbuf    Pointer to buffer where read data will be stored.
 * @param[in]   length  Number of bytes to read.
 * @return  Status of I2C comms.
 * @retval  I2CSTATUS_READWRITE_OK      Bridge successfully read from aXiom.
 * @retval  I2CSTATUS_COMMS_FAILED      Bridge could not communicate with aXiom (either through invalid setup or other error).
 * @retval  I2CSTATUS_DEVICE_TIMEOUT    aXiom did not respond to the bridge in time.
 */
uint32_t I2C_ReadAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length)
{
    // Wait until I2C bus is idle
    if (I2C_WaitForIdleBus() != eI2C_OK)
    {
        return I2CSTATUS_TIMEOUT;
    }

    // Prepare the transmit buffer
    uint8_t txbuffer[I2C_TXRX_BUFFER_SIZE] = {0U};
    if (I2C_PopulateCommandBytes(txbuffer, pagenum, offset, length, READ) != eI2C_OK)
    {
        // Pointer to buffer was NULL
        return I2CSTATUS_COMMS_FAILED;
    }

    // Send a start condition to I2C bus
    // Timeout in case another device on the I2C bus is hogging the bandwidth
    if (I2C_SendStartToBus() != eI2C_OK)
    {
        // Comms failed, send a stop condition and exit here
        I2C_SendStopToBus();
        return I2CSTATUS_TIMEOUT;
    }

    // Write to aXiom
    if (I2C_Transmit(txbuffer, NUM_CMD_BYTES) != eI2C_OK)
    {
        // Comms failed, send a stop condition and exit here
        I2C_SendStopToBus();
        return I2CSTATUS_TIMEOUT;
    }

    // Read data from aXiom
    // Repeated start and stop condition generated in I2C_Receive()
    en_i2cStatus i2c_status = I2C_Receive(pbuf, length);

    if (i2c_status == eI2C_FAIL)
    {
        return I2CSTATUS_COMMS_FAILED;
    }
    else if (i2c_status == eI2C_TIMEOUT)
    {
        return I2CSTATUS_TIMEOUT;
    }
    else
    {
        return I2CSTATUS_READWRITEOK;
    }
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
 * @brief   Waits for the I2C bus to become idle (i.e. waits for previous transmission to end), with a timeout.
 * @retval  I2C status: eI2C_Fail, eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_WaitForIdleBus(void)
{
    uint32_t timeout_counter = 0;
    en_i2cStatus status = eI2C_OK;

    while (i2c_flag_get(I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout_counter < I2C_MAX_TIMEOUT_COUNT))
    {
        delay_1us(I2C_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == I2C_MAX_TIMEOUT_COUNT)
    {
        status = eI2C_TIMEOUT;
    }

    return status;
}

/**
 * @brief   Populates the start of the buffer passed in with the axiom command bytes.
 * @param[in]   pBuffer     Pointer to buffer for commands bytes to be written to.
 * @param[in]   pagenum     Page address in axiom to be written to.
 * @param[in]   offset      Offset into page to be written to.
 * @param[in]   length      Number of bytes to write.
 * @param[in]   read_write  Field determining if we're requesting a read or write from aXiom.
 * @retval  I2C status: eI2C_Fail, eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_PopulateCommandBytes(uint8_t *pBuffer, uint8_t pagenum, uint8_t offset, uint32_t length, uint8_t read_write)
{
    en_i2cStatus status = eI2C_OK;

    if (pBuffer == NULL)
    {
        status = eI2C_FAIL;
    }
    else
    {
        pBuffer[0] = offset;
        pBuffer[1] = pagenum;
        pBuffer[2] = length & 0x00FFU;
        pBuffer[3] = ((length & 0xFF00U) >> 8U)| read_write;
    }

    return status;
}

/**
 * @brief   Sends the start condition to the I2C bus with a timeout.
 * @retval  I2C status: eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_SendStartToBus(void)
{
    en_i2cStatus status = eI2C_OK;
    uint32_t timeout_counter = 0;

    i2c_start_on_bus(I2C_PERIPH);
    while ((!i2c_flag_get(I2C_PERIPH, I2C_FLAG_SBSEND)) && (timeout_counter < I2C_MAX_TIMEOUT_COUNT))
    {
        delay_1us(I2C_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == I2C_MAX_TIMEOUT_COUNT)
    {
        // Clear flag (set as a result of device not responding/present)
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
        status = eI2C_TIMEOUT;
    }

    return status;
}

/**
 * @brief   Sends the stop condition to the I2C bus with a timeout.
 * @retval  I2C status: eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_SendStopToBus(void)
{
    en_i2cStatus status = eI2C_OK;
    uint32_t timeout_counter = 0;

    i2c_stop_on_bus(I2C_PERIPH);
    while ((I2C_CTL0(I2C_PERIPH) & I2C_CTL0_STOP) && (timeout_counter < I2C_MAX_TIMEOUT_COUNT))
    {
        delay_1us(I2C_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == I2C_MAX_TIMEOUT_COUNT)
    {
        // Clear flag (set as a result of device not responding/present)
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
        status = eI2C_TIMEOUT;
    }

    return status;
}

/**
 * @brief   Enacts the data write to the I2C bus.
 * @param[in]   pbuf    Pointer to the data to be sent.
 * @param[in]   length  Number of bytes to write.
 * @retval  I2C status: eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_Transmit(uint8_t *pbuf, uint32_t length)
{
    // Send slave address to I2C bus
    if (I2C_SendAddress(I2C_TRANSMITTER, ADDSEND_CLEAR) == eI2C_TIMEOUT)
    {
        // Target device not present/responsive
        return eI2C_TIMEOUT;
    }

    // Wait until the transmit data buffer is empty
    while (!i2c_flag_get(I2C_PERIPH, I2C_FLAG_TBE))
    {
        continue;
    }

    for (uint32_t i = 0U; i < length; i++)
    {
        if (I2C_TransmitByte(pbuf[i]) != eI2C_OK)
        {
            return eI2C_TIMEOUT;
        }
    }

    return eI2C_OK;
}

/**
 * @brief   Enacts the data read from the I2C bus.
 * @param[out]  pbuf    Pointer to buffer where read data will be stored.
 * @param[in]   length  Number of bytes to read.
 * @retval  I2C status: eI2C_Fail, eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_Receive(uint8_t *pbuf, uint32_t length)
{
    if (length == 0U)
    {
        // Do nothing
        // Makes sure we free the bus for other controllers/future transfers
        I2C_SendStopToBus();
        return eI2C_FAIL;
    }
    else
    {
        if (length == 2U)
        {
            // Configure ACKEN to control whether send ACK/NACK for the next byte, rather than the current one
            i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_NEXT);
        }

        // Repeated start
        if (I2C_SendStartToBus() != eI2C_OK)
        {
            return eI2C_TIMEOUT;
        }

        // Send slave address to I2C bus but don't clear the address sent flag yet
        if (I2C_SendAddress(I2C_RECEIVER, ADDSEND_DONTCLEAR) == eI2C_TIMEOUT)
        {
            // Target device not present/responsive
            I2C_SendStopToBus();
            return eI2C_TIMEOUT;
        }

        // Disable acknowledge before clearing ADDSEND flag if doing a short read
        if ((length == 1U) || (length == 2U))
        {
            i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
        }

        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);

        if (length == 1U)
        {
            I2C_SendStopToBus();

            // Flag gets set when the hardware I2C buffer is not empty
            if (I2C_WaitForRBNEFlag() != eI2C_OK)
            {
                I2C_SendStopToBus();
                return eI2C_TIMEOUT;
            }

            // Read data from I2C_DATA
            pbuf[0U] = i2c_data_receive(I2C_PERIPH);

            // Re-Enable acknowledge
            i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
        }
        else if (length == 2U)
        {
            // Wait for BTC to be set - indicates byte to receive but rx buffer is full
            if (I2C_WaitForBTCFlag() != eI2C_OK)
            {
                I2C_SendStopToBus();
                return eI2C_TIMEOUT;
            }

            I2C_SendStopToBus();

            // Read out the 2 data bytes
            for (uint32_t i = 0U; i < length; i++)
            {
                // Flag gets set when the hardware I2C buffer is not empty
                if (I2C_WaitForRBNEFlag() != eI2C_OK)
                {
                    I2C_SendStopToBus();
                    return eI2C_TIMEOUT;
                }

                // Read a byte from I2C_DATA
                pbuf[i] = i2c_data_receive(I2C_PERIPH);
            }

            // Revert ACKEN back to control whether send ACK/NACK for the CURRENT byte
            i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_CURRENT);

            // Re-Enable acknowledge
            i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
        }
        else
        {
            for (uint32_t i = 0U; i < length; i++)
            {
                if (i == (length - 3U))
                {
                    // Wait until the (length - 2) data byte is received into the shift register
                    if (I2C_WaitForBTCFlag() != eI2C_OK)
                    {
                        I2C_SendStopToBus();
                        return eI2C_TIMEOUT;
                    }

                    // Disable acknowledge
                    i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
                }

                // Flag gets set when the hardware I2C buffer is not empty
                if (I2C_WaitForRBNEFlag() != eI2C_OK)
                {
                    I2C_SendStopToBus();
                    return eI2C_TIMEOUT;
                }

                // Read a byte from I2C_DATA
                pbuf[i] = i2c_data_receive(I2C_PERIPH);
            }

            // Re-Enable acknowledge
            i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);

            I2C_SendStopToBus();
        }
    }

    return eI2C_OK;
}

/**
 * @brief   Sends a slave address to the I2C bus. If the timeout is reached, the function will return
 * @param[in]   read_write      Write or read from target device.
 *                              Must be one of the following arguments:
 * @arg                             I2C_TRANSMITTER - Write
 * @arg                             I2C_RECEIVER - Read
 * @param[in]   clear_addsend   Select if address sent flag is cleared in the function or not:
 * @arg                             ADDSEND_CLEAR - Address sent flag is cleared.
 * @arg                             Any other value - Address sent flag is not cleared.
 * @retval  I2C status: eI2C_Fail, eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_SendAddress(uint32_t read_write, uint8_t clear_addsend)
{
    uint32_t timeout_counter = 0U;
    en_i2cStatus status = eI2C_OK;

    i2c_master_addressing(I2C_PERIPH, I2C_GetI2CAddress(), read_write);

    // Wait for slave to acknowledge the address (address sent flag raised) or the timeout period to elapse
    while ((!i2c_flag_get(I2C_PERIPH, I2C_FLAG_ADDSEND)) && (timeout_counter < I2C_MAX_TIMEOUT_COUNT))
    {
        delay_1us(I2C_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == I2C_MAX_TIMEOUT_COUNT)
    {
        // Clear flags
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR); // Set as a result of device not responding/present
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);
        status = eI2C_TIMEOUT;
    }
    else
    {
        if (clear_addsend != 0U)
        {
            i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);
        }

        status = eI2C_OK;
    }

    return status;
}

/**
 * @brief   Transmits a byte of data on the I2C bus, with a timeout in the event of no response
 * @param[in]   data    Byte to be sent
 * @retval  I2C status: eI2C_Fail, eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_TransmitByte(uint8_t data)
{
    uint32_t timeout_counter = 0U;
    en_i2cStatus status = eI2C_OK;

    i2c_data_transmit(I2C_PERIPH, data);

    // Wait until the TBE bit is set
    while ((!i2c_flag_get(I2C_PERIPH, I2C_FLAG_TBE)) && (timeout_counter < I2C_MAX_TIMEOUT_COUNT))
    {
        delay_1us(I2C_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == I2C_MAX_TIMEOUT_COUNT)
    {
        // Clear flag (set as a result of device not responding/present)
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
        status = eI2C_TIMEOUT;
    }

    return status;
}

/**
 * @brief   Waits for the RBNE flag to be raised, with a timeout. Indicates the RX buffer is not empty.
 * @retval  I2C status: eI2C_Fail, eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_WaitForRBNEFlag(void)
{
    uint32_t timeout_counter = 0U;
    en_i2cStatus status = eI2C_OK;

    while ((!i2c_flag_get(I2C_PERIPH, I2C_FLAG_RBNE)) && (timeout_counter < I2C_MAX_TIMEOUT_COUNT))
    {
        delay_1us(I2C_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == I2C_MAX_TIMEOUT_COUNT)
    {
        // Clear flag (set as a result of device not responding/present)
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
        status = eI2C_TIMEOUT;
    }

    return status;
}

/**
 * @brief   Waits for the BTC flag to be raised, with a timeout. Indicates a byte can be received but the buffer is full.
 * @retval  I2C status: eI2C_Fail, eI2C_TIMEOUT or eI2C_OK.
 */
static en_i2cStatus I2C_WaitForBTCFlag(void)
{
    uint32_t timeout_counter = 0U;
    en_i2cStatus status = eI2C_OK;

    while ((!i2c_flag_get(I2C_PERIPH, I2C_FLAG_BTC)) && (timeout_counter < I2C_MAX_TIMEOUT_COUNT))
    {
        delay_1us(I2C_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == I2C_MAX_TIMEOUT_COUNT)
    {
        // Clear flag (set as a result of device not responding/present)
        i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
        status = eI2C_TIMEOUT;
    }

    return status;
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
 * @details aXiom typically has address 0x66 or 0x67.
 * @return  Address of connected device, in 7-bit format, left shifted by 1.
 */
static uint32_t FindI2CAddress(void)
{
    uint8_t     retryCount = 0;
    uint32_t    temp_addr;
    bool        addressFound = FALSE;

    // Waits for a maximum of ~12.5 seconds for aXiom to ACK an address request
    do
    {
        for (temp_addr = 0x66U; temp_addr < 0x68U; temp_addr++)
        {
            // Configure with the next i2c address
            i2c_mode_addr_config(I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, (temp_addr << 1U));

            // Some of these functions have timeout capability, however we are not concerned about the result,
            // as we are only interested when aXiom responds and not when it is not responsive (which it will be
            // until it has finished its start-up sequneces).

            // Wait until I2C bus is idle
            (void)I2C_WaitForIdleBus();

            // Send a start condition to I2C bus
            (void)I2C_SendStartToBus();

            // Send the current address to the device
            i2c_master_addressing(I2C_PERIPH, (temp_addr << 1U), I2C_TRANSMITTER);

            // If the address is correct the ADDSEND (address send) flag will be raised
            for (uint32_t timeout = 0; timeout <= MAX_ADDRSEND_ATTEMPTS; timeout++)
            {
                if (i2c_flag_get(I2C_PERIPH, I2C_FLAG_ADDSEND))
                {
                    // Pass
                    addressFound = TRUE;

                    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_ADDSEND);
                    break;
                }

                if (timeout == MAX_ADDRSEND_ATTEMPTS)
                {
                    // Fail
                    // Need to clear this flag as there will have been an ACK error
                    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
                    break;
                }

                // Gives the ADDSEND flag time to assert
                delay_1us(I2C_SLEEP_US);
            }

            // Send a stop condition to I2C bus and wait for stop condition to be generated
            // If we had a timeout when sending a STOP, the hardware seems to get confused and needs to be reset
            // otherwise it never sees aXiom.
            if (I2C_SendStopToBus() != eI2C_OK)
            {
                // Reset I2C hardware
                i2c_disable(I2C_PERIPH);
                i2c_deinit(I2C_PERIPH);

                // Configure the peripheral settings
                i2c_clock_config(I2C_PERIPH, I2C_FASTMODE, I2C_DTCY_2);
                i2c_enable(I2C_PERIPH);
                i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
            }

            if (addressFound == TRUE)
            {
                // aXiom found - break out of temp_addr for() loop
                break;
            }
        }

        // Sleep for a bit before trying again
        delay_1ms(50U);
        retryCount++;
    } while ((addressFound == FALSE) && (retryCount < MAX_ADDR_SEARCH_ATTEMPTS));

    if (addressFound == FALSE)
    {
        temp_addr = NO_AXIOM_FOUND_ADDR;
    }

    return (temp_addr << 1U);
}
