/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       spi.c
 * \brief      SPI hardware driver
 *
 * \version    1.0
 * \date       Feb 12 2010
 * \author     Miguel Luis
 */
 
//#include "stm32f10x_spi.h"
#include "Board.h"
#include "spi.h"

#define SPI_BIT_RATE              4000000

/* -----------------------------------------------------------------------------
*  Local variables
* ------------------------------------------------------------------------------
*/
static PIN_Config BoardFlashPinTable[] =
{
    Board_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN, /* Ext. flash chip select */

    PIN_TERMINATE
};

static PIN_Handle hRadioPin = NULL;
static PIN_State pinState;

// SPI interface
static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;

/* -----------------------------------------------------------------------------
*  Local function
* ------------------------------------------------------------------------------
*/
static bool Spi_open(uint32_t bitRate);

/* -----------------------------------------------------------------------------
*  SpiInit
* ------------------------------------------------------------------------------
*/
bool SpiInit( void )
{
    bool f;

    SPI_init();
    f = Spi_open(SPI_BIT_RATE);
    
    return f;
}

/*******************************************************************************
* @fn          Spi_open
*
* @brief       Open the RTOS SPI driver
*
* @param       bitRate - transfer speed in bits/sec
*
* @return      true if success
*/
static bool Spi_open(uint32_t bitRate)
{
    /*  Configure SPI as master */
    SPI_Params_init(&spiParams);
    spiParams.bitRate = bitRate;
    spiParams.dataSize = 8; 
    spiParams.frameFormat = SPI_POL0_PHA0;           //相位0极性0
    spiParams.mode = SPI_MASTER;                     //SPI主从模式
    spiParams.transferCallbackFxn = NULL;
    spiParams.transferMode = SPI_MODE_BLOCKING;      //阻塞
    spiParams.transferTimeout = SPI_WAIT_FOREVER;

    /* Attempt to open SPI. */
    spiHandle = SPI_open(Board_SPI0, &spiParams);

    return spiHandle != NULL;
}

/*******************************************************************************
* @fn          Spi_close
*
* @brief       Close the RTOS SPI driver
*
* @return      none
*/
static void Spi_close(void)
{
    if (spiHandle != NULL)
    {
        // Close the RTOS driver
        SPI_close(spiHandle);
        spiHandle = NULL;
    }
}

/*******************************************************************************
* @fn          Spi_write
*
* @brief       Write to an SPI device
*
* @param       buf - pointer to data buffer
* @param       len - number of bytes to write
*
* @return      '0' if success, -1 if failed
*/
static int Spi_write(const uint8_t *buf, size_t len)
{
    SPI_Transaction masterTransaction;

    masterTransaction.count  = len;
    masterTransaction.txBuf  = (void*)buf;
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = NULL;

    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}


/*******************************************************************************
* @fn          Spi_read
*
* @brief       Read from an SPI device
*
* @param       buf - pointer to data buffer
* @param       len - number of bytes to write
*
* @return      '0' if success, -1 if failed
*/
static int Spi_read(uint8_t *buf, size_t len)
{
    SPI_Transaction masterTransaction;

    masterTransaction.count = len;
    masterTransaction.txBuf = NULL;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = buf;

    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
}

uint8_t SpiInOut( uint8_t outData )
{
  
    /* Send SPIy data */
//    SPI_I2S_SendData( SPI_INTERFACE, outData );
//    while( SPI_I2S_GetFlagStatus( SPI_INTERFACE, SPI_I2S_FLAG_RXNE ) == RESET );
//    return SPI_I2S_ReceiveData( SPI_INTERFACE );
    SPI_Transaction masterTransaction;
    uint8_t txbuf[2] = {0}; 
    uint8_t rxbuf[2] = {0}; 
    
    txbuf[0] = outData;
    masterTransaction.count = 1;
    masterTransaction.txBuf = txbuf;
    masterTransaction.arg = NULL;
    masterTransaction.rxBuf = rxbuf;
    SPI_transfer(spiHandle, &masterTransaction);
    
    return rxbuf[0];
}

