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
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
//#include <stdint.h>
//#include <stdbool.h> 

#include "inc/hw_gpio.h"
#include "board.h"
#include "platform.h"
#include "hw_spi.h"

#if defined( USE_SX1276_RADIO )

//#include "ioe.h"
#include "spi.h"
#include "../../radio/sx1276-Hal.h"

/*!
 * SX1276 SPI NSS I/O definitions
 */
#define NSS_PIN                                     IOID_12

/*!
 * SX1276 DIO pins  I/O definitions
 */
//#define DIO0_PIN                                    IOID_14
#define DIO0_PIN                                    IOID_7
#define DIO1_PIN                                    IOID_0
#define DIO2_PIN                                    IOID_0
#define DIO3_PIN                                    IOID_13
#define DIO4_PIN                                    IOID_0
#define DIO5_PIN                                    IOID_0 
#define RXTX_PIN                                    IOID_0

/*!
 * SX1276 RESET I/O definitions
 */
#define RESET_PIN                                   IOID_4
#define RESET_HIGH()                                PIN_setOutputValue(radCtrlHandle, RESET_PIN, 1);
#define RESET_LOW()                                 PIN_setOutputValue(radCtrlHandle, RESET_PIN, 0);

#define RXE_PIN                                     IOID_13
#define RXE_HIGH()                                  PIN_setOutputValue(radCtrlHandle, RXE_PIN, 1);
#define RXE_LOW()                                   PIN_setOutputValue(radCtrlHandle, RXE_PIN, 0);
#define RXE_STATE()                                 PIN_getOutputValue(RXE_PIN)

#define TXE_PIN                                    IOID_13
#define TXE_HIGH()                                 PIN_setOutputValue(radCtrlHandle, TXE_PIN, 1);
#define TXE_LOW()                                  PIN_setOutputValue(radCtrlHandle, TXE_PIN, 0);
#define TXE_STATE()                                PIN_getOutputValue(TXE_PIN)

#define NSS_HIGH()                                 PIN_setOutputValue(radCtrlHandle, NSS_PIN, 1);
#define NSS_LOW()                                  PIN_setOutputValue(radCtrlHandle, NSS_PIN, 0);

static PIN_Config radCtrlCfg[] =
{
//  RXE_PIN  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  TXE_PIN  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  NSS_PIN  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  RESET_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  
  DIO0_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_NOPULL,
//  DIO1_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
//  DIO2_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
//  DIO3_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
//  DIO4_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
//  DIO5_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
  PIN_TERMINATE
};
static PIN_Handle radCtrlHandle;
static PIN_State  radCtrlState;


void Soft_delay_ms(uint16_t time)
{    
   uint16_t i=0;
   
   while(time--)
   {
      i=7950;
      while(i--) ;    
   }
}

void Set_RF_Switch_RX(void)
{
//    TXE_LOW();
}

void Set_RF_Switch_TX(void)
{
//    TXE_HIGH();
}

void SX1276InitIo( void )
{
    radCtrlHandle = PIN_open(&radCtrlState, radCtrlCfg);

     // Configure SPI-->NSS as output
    NSS_HIGH();
	
    //默认设设置为接收状态
    Set_RF_Switch_RX();	
	
    // Configure radio DIO as inputs
}

void SX1276SetReset( uint8_t state )
{
    if( state == RADIO_RESET_ON )
    {
        RESET_LOW();
    }
    else
    {
        RESET_HIGH();
    }
}


void SX1276SetNss( uint8_t state )
{
    if( state )
    {
        NSS_HIGH();
    }
    else
    {
        NSS_LOW();
    }
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    NSS_LOW();

    SpiInOut( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    //NSS = 1;
    NSS_HIGH();
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    NSS_LOW();

    SpiInOut( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }

    //NSS = 1;
    NSS_HIGH();
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

inline uint_t SX1276ReadDio0( void )
{
    return PIN_getInputValue(DIO0_PIN);
}

inline uint_t SX1276ReadDio1( void )
{
    return 0;
//    return PIN_getInputValue(DIO1_PIN);
}

inline uint_t SX1276ReadDio2( void )
{
    return 0;//PIN_getInputValue(DIO2_PIN);
}

inline uint_t SX1276ReadDio3( void )
{
//    return PIN_getInputValue(DIO3_PIN);
    return 0;
}

inline uint_t SX1276ReadDio4( void )
{
    return 0;//PIN_getInputValue(DIO4_PIN);
}

inline uint_t SX1276ReadDio5( void )
{
    return 0;//PIN_getInputValue(DIO5_PIN);
}


//射频芯片收发切换
inline void SX1276WriteRxTx( uint8_t txEnable )
{
    if( txEnable != 0 )
    {
		Set_RF_Switch_TX(); //单片机将射频开关芯片切换成发射状态
//        IoePinOn( FEM_CTX_PIN );
//        IoePinOff( FEM_CPS_PIN );
    }
    else
    {
		Set_RF_Switch_RX();  //单片机将射频开关芯片切换成接收状态
//        IoePinOff( FEM_CTX_PIN );
//        IoePinOn( FEM_CPS_PIN );
    }
}

#endif // USE_SX1276_RADIO
