/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    SPI_interface.h    >>>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : SPI                                             **
 **                                                                           **
 **===========================================================================**
 */
#ifndef _SPI_INTERFACE_H_
#define _SPI_INTERFACE_H_

#include "stdint.h"


typedef enum
{
  SPI_CHANNEL1,
  SPI_CHANNEL2,
  SPI_CHANNEL3,
  SPI_CHANNEL4,
} SPI_Channel_t;

typedef enum
{
  SPI_CPHA_1EDGE,
  SPI_CPHA_2EDGE,
} SPI_CPHA_t;

typedef enum
{
  SPI_CPOL_LOW,
  SPI_CPOL_HIGH,
} SPI_CPOL_t;

typedef enum
{
  SPI_MODE_SLAVE,
  SPI_MODE_MASTER,
} SPI_Mode_t;

typedef enum
{
  SPI_BAUDRATEPRESCALER_2,
  SPI_BAUDRATEPRESCALER_4,
  SPI_BAUDRATEPRESCALER_8,
  SPI_BAUDRATEPRESCALER_16,
  SPI_BAUDRATEPRESCALER_32,
  SPI_BAUDRATEPRESCALER_64,
  SPI_BAUDRATEPRESCALER_128,
  SPI_BAUDRATEPRESCALER_256,
} SPI_BAUDRATEPRESCALER_t;

typedef enum
{
  SPI_SPE_DIS,
  SPI_SPE_EN,
} SPI_SPE_t;

typedef enum
{
  SPI_MSBFIRST,
  SPI_LSBFIRST,
} SPI_BFIRST_t;

typedef enum
{
  SPI_NSS_HARDWARE,
  SPI_NSS_SOFTWARE,
} SPI_NSS_MAN_t;

typedef enum
{
  SPI_NSSI_SELECT,
  SPI_NSSI_NOT_SELECT,
} SPI_NSSI_MODE_t;

typedef enum
{
  SPI_RXONLY_DISABLE,  /* Full Dublex Mode */
  SPI_RXONLY_ENABLE,   /* Recieve only Mode */
} SPI_RXONLY_t;

typedef enum
{
  SPI_DFF_8BIT,
  SPI_DFF_16BIT,
} SPI_DFF_t;

typedef enum
{
  SPI_CRCNEXT_DISABLE,
  SPI_CRCNEXT_ENABLE,
} SPI_CRCNEXT_t;

typedef enum
{
  SPI_CRCDIS,
  SPI_CRCEN,
} SPI_CRCEN_t;

typedef enum
{
  SPI_UNIDIMODE,
  SPI_BIDIMODE,
} SPI_DIMODE_t;

typedef enum
{
  SPI_OUTPUT_DIS,
  SPI_OUTPUT_EN,
} SPI_BIDIOE_t;

typedef struct
{
  SPI_Channel_t Channel;
  SPI_CPHA_t CPHA;
  SPI_CPOL_t CPOL;
  SPI_Mode_t Mode;
  SPI_BAUDRATEPRESCALER_t BaudRatePrescaler;
  SPI_SPE_t SPE;
  SPI_BFIRST_t BFIRST;
  SPI_NSS_MAN_t NSS_MAN;
  SPI_NSSI_MODE_t NSSI_MODE;
  SPI_RXONLY_t RXONLY;
  SPI_DFF_t DFF;
  SPI_CRCEN_t CRC_MODE;
  SPI_CRCNEXT_t CRCNEXT;
  SPI_DIMODE_t DIMODE;
  SPI_BIDIOE_t BIDIOE;
} SPI_Config_t;

//SPI_Config_t SPI1 =
//{
//    .Channel = SPI_CHANNEL1,
//    .CPHA = SPI_CPHA_1EDGE,
//    .CPOL = SPI_CPOL_HIGH,
//    .Mode = SPI_MODE_MASTER,
//    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
//    .SPE = SPI_SPE_EN,
//    .BFIRST = SPI_MSBFIRST,
//    .NSS_MAN = SPI_NSS_SOFTWARE,
//    .NSSI_MODE = SPI_NSSI_NOT_SELECT,
//    .RXONLY = SPI_RXONLY_DISABLE,
//    .DFF = SPI_DFF_8BIT,
//    .CRC_MODE = SPI_CRCDIS,
//    .DIMODE = SPI_UNIDIMODE,
//};

ErrorState_t SPI_enumInit(SPI_Config_t *ChannelConfig);
ErrorState_t SPI_enumTrancieve(SPI_Config_t *ChannelConfig, uint16_t TX_Data, uint16_t *RX_Data);
ErrorState_t SPI_enumTransmit(SPI_Config_t *ChannelConfig, uint16_t TX_Data);
ErrorState_t SPI_enumReceive(SPI_Config_t *ChannelConfig, uint16_t *RX_Data);
#endif /* _SPI_INTERFACE_H_ */
