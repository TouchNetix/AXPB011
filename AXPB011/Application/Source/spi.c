/*!
 @file  spi.c
 @brief SPI driver file.

 @version 25/08/2022 - V01.00. First release. Author: James Cameron.
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
#include "spi.h"
#include "systick.h"

/*******************************************************************************
 * File Scope Conditional Compilation Flags
 ******************************************************************************/


/*******************************************************************************
 * File Scope Data Types
 ******************************************************************************/
typedef enum
{
    eSPI_OK         = 0,
    eSPI_FAIL       = 1,
    eSPI_TIMEOUT    = 2,
} en_spiStatus;

/*******************************************************************************
 * Exported Constants
 ******************************************************************************/


/*******************************************************************************
 * File Scope Constants
 ******************************************************************************/
#define SPI_PERIPH  SPI0

// Timeout waits for a maximum of 250us - more than enough time for transfers to complete but shouldn't be taking longer than this
// SPI speed = 4MHz
// No. bytes = 4 + 32 + 58 = 94
// Time for max. transfer = 23.5us + overhead, dma set, etc.
#define SPI_MAX_TIMEOUT_COUNT   (50U)
#define SPI_SLEEP_US            (  5U)

#define SPI_NSS_GPIO_PORT       (GPIOA)
#define SPI_SCK_GPIO_PORT       (GPIOA)
#define SPI_MISO_GPIO_PORT      (GPIOA)
#define SPI_MOSI_GPIO_PORT      (GPIOA)
#define SPI_NSS_PIN             (GPIO_PIN_4)
#define SPI_SCK_PIN             (GPIO_PIN_5)
#define SPI_MISO_PIN            (GPIO_PIN_6)
#define SPI_MOSI_PIN            (GPIO_PIN_7)

#define NUM_CMD_BYTES           (4U)
#define NUM_PADDING_BYTES       (32U)
#define USB_ENDPOINT_SIZE       (58U)   // max USB packet size (64) minus 2 bridge header bytes and 4 axiom command bytes
#define SPI_TXRX_BUFFER_SIZE    (NUM_CMD_BYTES + NUM_PADDING_BYTES + USB_ENDPOINT_SIZE)
#define WRITE                   (0x00U)
#define READ                    (0x80U)

#define SPI_COOLOFF_US          (200U)  // Anything less than this and SPI doesn't function correctly - reads from aXiom too quickly

/*******************************************************************************
 * File Scope Variables
 ******************************************************************************/
uint8_t g_spi_tx_buffer[SPI_TXRX_BUFFER_SIZE] = {0};
uint8_t g_spi_rx_buffer[SPI_TXRX_BUFFER_SIZE] = {0};

/*******************************************************************************
 * Exported Variables
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
static void         SPI_PinConfig(void);
static void         SPI_DMAConfig(void);
static void         SPI_SetNumTxBytes(uint16_t length);
static en_spiStatus SPI_WaitForTransferToComplete(void);

/*******************************************************************************
 * Exported Function Definitions
 ******************************************************************************/
/**
 * @brief   Configure the SPI peripheral
 */
void SPI_Init(void)
{
    // Enable peripheral clock
    rcu_periph_clock_enable(RCU_SPI0);

    // Configure the pins used for SPI
    SPI_PinConfig();

    // Configure the DMA for SPI
    SPI_DMAConfig();

    // Initialise SPI parameters with defaults
    spi_parameter_struct spi_init_struct;
    spi_struct_para_init(&spi_init_struct);

    // Configure the SPI parameters
    // SPI0 is connected to APB2. The system clock is running at 96 MHz.
    // APB2 = SYSCLK/4 = 24 MHz.
    // SPI speed = APB2/8 = 3 MHz
    spi_init_struct.prescale                = SPI_PSC_8;
    spi_init_struct.device_mode             = SPI_MASTER;
    spi_init_struct.trans_mode              = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.frame_size              = SPI_FRAMESIZE_8BIT;
    spi_init_struct.nss                     = SPI_NSS_SOFT;
    spi_init_struct.endian                  = SPI_ENDIAN_MSB;
    spi_init_struct.clock_polarity_phase    = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init(SPI0, &spi_init_struct);

    spi_enable(SPI_PERIPH);
}

/**
 * @brief
 */
void SPI_DeInit(void)
{
    spi_disable(SPI_PERIPH);

    // Clear the SPI peripheral settings
    rcu_periph_reset_enable(RCU_SPI0RST);
    rcu_periph_reset_disable(RCU_SPI0RST);
    gpio_deinit(SPI_SCK_GPIO_PORT);
    gpio_deinit(SPI_MISO_GPIO_PORT);
    gpio_deinit(SPI_MOSI_GPIO_PORT);
    gpio_deinit(SPI_NSS_GPIO_PORT);

    // Clear the DMA settings
    dma_deinit(DMA_CH1);
    dma_deinit(DMA_CH2);

    // Last step - disable the peripheral clocks
    rcu_periph_clock_disable(RCU_SPI0);
    rcu_periph_clock_disable(RCU_DMA);
}

/**
 * @brief   Send data to the connected aXiom device via SPI.
 * @param[in]   pagenum Page address in axiom to be written to.
 * @param[in]   offset  Offset into page to be written to.
 * @param[in]   pbuf    Pointer to the data to be sent.
 * @param[in]   length  Number of bytes to write.
 * @return  Status of SPI comms.
 * @retval  SPISTATUS_WRITE_OK_NOREAD   Bridge successfully wrote to aXiom.
 * @retval  SPISTATUS_COMMS_FAILED      Bridge could not communicate with aXiom (either through invalid setup or other error).
 * @retval  SPISTATUS_DEVICE_TIMEOUT    Bridge got stuck waiting for DMA transfers to complete (likely an invalid setup).
 */
uint32_t SPI_WriteAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length)
{
    uint32_t status = SPISTATUS_WRITE_OK_NOREAD;

    memset(g_spi_tx_buffer, 0x00, SPI_TXRX_BUFFER_SIZE);
    memset(g_spi_rx_buffer, 0x00, SPI_TXRX_BUFFER_SIZE);

    // Setup buffer to write to axiom
    g_spi_tx_buffer[0] = offset;
    g_spi_tx_buffer[1] = pagenum;
    g_spi_tx_buffer[2] = length & 0x00FFU;
    g_spi_tx_buffer[3] = ((length & 0xFF00U) >> 8U)| WRITE;
    memcpy(&g_spi_tx_buffer[NUM_CMD_BYTES + NUM_PADDING_BYTES], pbuf, length);

    // Configure the DMA length
    SPI_SetNumTxBytes(NUM_CMD_BYTES + NUM_PADDING_BYTES + length);

    // Set the nSS signal low (assert)
    gpio_bit_write(SPI_NSS_GPIO_PORT, SPI_NSS_PIN, RESET);

    // Start transfer
    spi_dma_enable(SPI_PERIPH, SPI_DMA_RECEIVE);
    spi_dma_enable(SPI_PERIPH, SPI_DMA_TRANSMIT);

    // Wait for the DMA transfer to complete
    // This function can only return eSPI_OK or eSPI_TIMEOUT
    if (SPI_WaitForTransferToComplete() == eSPI_TIMEOUT)
    {
        status = SPISTATUS_TIMEOUT;
    }

    // Set the nSS signal high (de-assert)
    gpio_bit_write(SPI_NSS_GPIO_PORT, SPI_NSS_PIN, SET);

    // Stop transfers and clear flags
    spi_dma_disable(SPI_PERIPH, SPI_DMA_TRANSMIT);
    spi_dma_disable(SPI_PERIPH, SPI_DMA_RECEIVE);
    dma_flag_clear(DMA_CH2, DMA_INT_FLAG_FTF);
    dma_flag_clear(DMA_CH1, DMA_INT_FLAG_FTF);

    // Delay to prevent reading in quick succession. Need to give aXiom time to setup the next transfer.
    // Even with this delay the bridge could read ~5000 reports per second (although probably less in practice)
    // which is far more than would ever be required.
    // Only required for SPI.
    delay_1us(SPI_COOLOFF_US);

    return status;
}

/**
 * @brief   Receive data from the connected aXiom device via SPI.
 * @details 32 bytes of data are sent after the command bytes to allow axiom
 *          time to prepare a response.
 * @param[in]   pagenum Page address in axiom to be read from.
 * @param[in]   offset  Offset into page to be read from.
 * @param[out]  pbuf    Pointer to buffer where read data will be stored.
 * @param[in]   length  Number of bytes to read.
 * @return  Status of SPI comms.
 * @retval  SPISTATUS_READWRITEOK      Bridge successfully read from aXiom.
 * @retval  SPISTATUS_COMMS_FAILED      Bridge could not communicate with aXiom (either through invalid setup or other error).
 * @retval  SPISTATUS_DEVICE_TIMEOUT    Bridge got stuck waiting for DMA transfers to complete (likely an invalid setup).
 */
uint32_t SPI_ReadAxiom(uint8_t pagenum, uint8_t offset, uint8_t *pbuf, uint32_t length)
{
    uint32_t status = SPISTATUS_READWRITEOK;

    if (length != 0U)
    {
        memset(&g_spi_tx_buffer[0], 0x00, SPI_TXRX_BUFFER_SIZE);
        memset(&g_spi_rx_buffer[0], 0x00, SPI_TXRX_BUFFER_SIZE);

        // Setup buffer to write to axiom
        g_spi_tx_buffer[0] = offset;
        g_spi_tx_buffer[1] = pagenum;
        g_spi_tx_buffer[2] = length & 0x00FFU;
        g_spi_tx_buffer[3] = ((length & 0xFF00U) >> 8U)| READ;

        // Configure the DMA length
        SPI_SetNumTxBytes(NUM_CMD_BYTES + NUM_PADDING_BYTES + length);

        // Set the nSS signal low (assert)
        gpio_bit_write(SPI_NSS_GPIO_PORT, SPI_NSS_PIN, RESET);

        // Start transfer
        spi_dma_enable(SPI_PERIPH, SPI_DMA_RECEIVE);
        spi_dma_enable(SPI_PERIPH, SPI_DMA_TRANSMIT);

        // Wait for the DMA transfer to complete
        // This function can only return eSPI_OK or eSPI_TIMEOUT
        if (SPI_WaitForTransferToComplete() == eSPI_TIMEOUT)
        {
            status = SPISTATUS_TIMEOUT;
        }

        // Set the nSS signal high (de-assert)
        gpio_bit_write(SPI_NSS_GPIO_PORT, SPI_NSS_PIN, SET);

        // Stop transfers and clear interrupt flags
        spi_dma_disable(SPI_PERIPH, SPI_DMA_TRANSMIT);
        spi_dma_disable(SPI_PERIPH, SPI_DMA_RECEIVE);
        dma_flag_clear(DMA_CH2, DMA_INT_FLAG_FTF);
        dma_flag_clear(DMA_CH1, DMA_INT_FLAG_FTF);

        // Read data is received after the command and padding bytes, so copy from there
        memcpy(&pbuf[0], &g_spi_rx_buffer[NUM_CMD_BYTES + NUM_PADDING_BYTES], length);

        // Delay to prevent reading in quick succession. Need to give aXiom time to setup the next transfer.
        // Even with this delay the bridge could read ~5000 reports per second (although probably less in practice)
        // which is far more than would ever be required.
        // Only required for SPI.
        delay_1us(SPI_COOLOFF_US);
    }
    else
    {
        // Incorrect length requested
        status = SPISTATUS_COMMS_FAILED;
    }

    return status;
}

/*******************************************************************************
 * File Scope Function Definitions
 ******************************************************************************/
static void SPI_PinConfig(void)
{
    // Configure SPI0 pins
    gpio_af_set(SPI_SCK_GPIO_PORT , GPIO_AF_0, SPI_SCK_PIN );
    gpio_af_set(SPI_MISO_GPIO_PORT, GPIO_AF_0, SPI_MISO_PIN);
    gpio_af_set(SPI_MOSI_GPIO_PORT, GPIO_AF_0, SPI_MOSI_PIN);

    gpio_mode_set(SPI_SCK_GPIO_PORT , GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK_PIN );
    gpio_mode_set(SPI_MISO_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_MISO_PIN);
    gpio_mode_set(SPI_MOSI_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_MOSI_PIN);

    gpio_output_options_set(SPI_SCK_GPIO_PORT , GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPI_SCK_PIN );
    gpio_output_options_set(SPI_MISO_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPI_MISO_PIN);
    gpio_output_options_set(SPI_MOSI_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPI_MOSI_PIN);

    // Configure nSS/PA4 - instantiate high
    GPIO_OCTL(SPI_NSS_GPIO_PORT) |= SPI_NSS_PIN;
    gpio_mode_set(SPI_NSS_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, SPI_NSS_PIN);
    gpio_output_options_set(SPI_NSS_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, SPI_NSS_PIN);
}

static void SPI_DMAConfig(void)
{
    dma_parameter_struct dma_init_struct;

    dma_deinit(DMA_CH2);
    dma_deinit(DMA_CH1);

    // Initialise with defaults
    dma_struct_para_init(&dma_init_struct);

    // Parameters common to both DMA channels
    dma_init_struct.periph_addr = (uint32_t)&SPI_DATA(SPI_PERIPH);
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.number       = 0;   // Set per transfer

    // Configure SPI0 transmit DMA: DMA_CH2
    dma_init_struct.memory_addr  = (uint32_t)g_spi_tx_buffer;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init(DMA_CH2, &dma_init_struct);
    dma_circulation_disable(DMA_CH2);
    dma_memory_to_memory_disable(DMA_CH2);

    // Configure SPI0 receive DMA: DMA_CH1
    dma_init_struct.memory_addr = (uint32_t)g_spi_rx_buffer;
    dma_init_struct.direction   = DMA_PERIPHERAL_TO_MEMORY;
    dma_init(DMA_CH1, &dma_init_struct);
    dma_circulation_disable(DMA_CH1);
    dma_memory_to_memory_disable(DMA_CH1);
}

static void SPI_SetNumTxBytes(uint16_t length)
{
    // DMA config setup be altered if CHEN is 1
    dma_channel_disable(DMA_CH1);
    dma_channel_disable(DMA_CH2);

    // Configure the number of transfers to perform for the next DMA transfer
    DMA_CHCNT(DMA_CH2) = (uint32_t)(length & DMA_CHANNEL_CNT_MASK);
    DMA_CHCNT(DMA_CH1) = (uint32_t)(length & DMA_CHANNEL_CNT_MASK);

    dma_channel_enable(DMA_CH1);
    dma_channel_enable(DMA_CH2);
}

static en_spiStatus SPI_WaitForTransferToComplete(void)
{
    uint32_t timeout_counter = 0U;
    en_spiStatus status = eSPI_OK;

    while((dma_flag_get(DMA_CH2, DMA_INT_FLAG_FTF) == RESET) && (timeout_counter < SPI_MAX_TIMEOUT_COUNT))
    {
        delay_1us(SPI_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == SPI_MAX_TIMEOUT_COUNT)
    {
        status = eSPI_TIMEOUT;
    }

    timeout_counter = 0U;
    while((dma_flag_get(DMA_CH1, DMA_INT_FLAG_FTF) == RESET) && (timeout_counter < SPI_MAX_TIMEOUT_COUNT))
    {
        delay_1us(SPI_SLEEP_US);
        timeout_counter++;
        continue;
    }

    if (timeout_counter == SPI_MAX_TIMEOUT_COUNT)
    {
        status = eSPI_TIMEOUT;
    }

    return status;
}
