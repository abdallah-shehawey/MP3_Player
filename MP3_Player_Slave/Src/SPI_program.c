/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    SPI_program.c    >>>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : SPI                                             **
 **                                                                           **
 **===========================================================================**
 */
#include "stdint.h"
#include "ErrTypes.h"
#include "STM32F446xx.h"

#include "SPI_private.h"
#include "SPI_config.h"
#include "SPI_interface.h"

/* Array of SPI port register definitions for easy access */
static SPI_RegDef_t *SPI_Channel[SPI_CHANNEL_COUNT] = {MSPI1, MSPI2, MSPI3, MSPI4};
/*Global flag for the SPI Busy State*/
static uint8_t SPI_u8State = IDLE;


ErrorState_t SPI_enumInit(SPI_Config_t *ChannelConfig)
{
  ErrorState_t Local_u8ErrorState = OK;

  if (ChannelConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    if(ChannelConfig->Channel > SPI_CHANNEL_COUNT)
    {
      Local_u8ErrorState = NOK;
    }
    else
    {
      /* Configure CLOCK PHASE */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->CPHA & 0X1) << CR1_CPHA;
      /* Configure Clock Polarity */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->CPOL & 0X1) << CR1_CPOL;
      /* Configure Master/Slave Mode */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->Mode & 0X1) << CR1_MSTR;
      /* Configure Baud Rate */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->BaudRatePrescaler & 0X7) << CR1_BR;
      /* Configure which bit first -> MSB or LSB */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->BFIRST & 0X1) << CR1_LSBFIRST;
      /* Configure NSS Management */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->NSS_MAN & 0X1) << CR1_SSM;
      if (ChannelConfig->NSS_MAN == SPI_NSS_SOFTWARE)
      {
        /* Configure NSS Mode */
        SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->NSSI_MODE & 0X1) << CR1_SSI;
      }
      /* Configure Recieve Only Mode */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->RXONLY & 0X1) << CR1_RXONLY;
      /* Configure Data Frame Format */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->DFF & 0X1) << CR1_DFF;
      /* Configure CRC Mode */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->CRC_MODE & 0X1) << CR1_CRCEN;
      if (ChannelConfig->CRC_MODE == SPI_CRCEN)
      {
        /* Configure CRC Next */
        SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->CRCNEXT & 0X1) << CR1_CRCNEXT;
      }
      /* Configure Bidirectional Mode */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->DIMODE & 0X1) << CR1_BIDIMODE;
      if (ChannelConfig->DIMODE == SPI_BIDIMODE)
      {
        /* Configure Bidirectional Output Enable */
        SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->BIDIOE & 0X1) << CR1_BIDIOE;
      }
      /* Configure SPI Enable */
      SPI_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->SPE & 0X1) << CR1_SPE;
    }
  }
  return Local_u8ErrorState;
}

ErrorState_t SPI_enumTrancieve(SPI_Config_t *ChannelConfig, uint16_t TX_Data, uint16_t *RX_Data)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint32_t Local_u32TimeoutCounter = 0;

  if (ChannelConfig == NULL || RX_Data == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    if (ChannelConfig->Channel > SPI_CHANNEL_COUNT)
    {
      Local_u8ErrorState = NOK;
    }
    else
    {
      if (SPI_u8State == IDLE)
      {
        SPI_u8State = BUSY;
        while((SPI_Channel[ChannelConfig->Channel]->SR & (1 << SR_TXE)) >> SR_TXE == 0 && (Local_u32TimeoutCounter != SPI_u32TIMEOUT))
        {
          Local_u32TimeoutCounter++;
        }
        if (Local_u32TimeoutCounter == SPI_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
        else
        {
          SPI_Channel[ChannelConfig->Channel]->DR = TX_Data;
        }
        Local_u32TimeoutCounter = 0;
        while((SPI_Channel[ChannelConfig->Channel]->SR & (1 << SR_RXNE)) >> SR_RXNE == 0 && (Local_u32TimeoutCounter != SPI_u32TIMEOUT))
        {
          Local_u32TimeoutCounter++;
        }
        if (Local_u32TimeoutCounter == SPI_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
        else
        {
          *RX_Data = SPI_Channel[ChannelConfig->Channel]->DR;
        }
        SPI_u8State = IDLE;
      }
    }
  }
  return Local_u8ErrorState;
}


ErrorState_t SPI_enumTransmit(SPI_Config_t *ChannelConfig, uint16_t TX_Data)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint32_t Local_u32TimeoutCounter = 0;

  if (ChannelConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    if (ChannelConfig->Channel > SPI_CHANNEL_COUNT)
    {
      Local_u8ErrorState = NOK;
    }
    else
    {
      if (SPI_u8State == IDLE)
      {
        SPI_u8State = BUSY;
        while((SPI_Channel[ChannelConfig->Channel]->SR & (1 << SR_TXE)) >> SR_TXE == 0 && (Local_u32TimeoutCounter != SPI_u32TIMEOUT))
        {
          Local_u32TimeoutCounter++;
        }
        if (Local_u32TimeoutCounter == SPI_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
        else
        {
          SPI_Channel[ChannelConfig->Channel]->DR = TX_Data;
        }
        SPI_u8State = IDLE;
      }
    }
  }
  return Local_u8ErrorState;
}


ErrorState_t SPI_enumReceive(SPI_Config_t *ChannelConfig, uint16_t *RX_Data)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint32_t Local_u32TimeoutCounter = 0;

  if (ChannelConfig == NULL || RX_Data == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    if (ChannelConfig->Channel > SPI_CHANNEL_COUNT)
    {
      Local_u8ErrorState = NOK;
    }
    else
    {
      if (SPI_u8State == IDLE)
      {
        SPI_u8State = BUSY;
        while((SPI_Channel[ChannelConfig->Channel]->SR & (1 << SR_RXNE)) >> SR_RXNE == 0 && (Local_u32TimeoutCounter != SPI_u32TIMEOUT))
        {
          Local_u32TimeoutCounter++;
        }
        if (Local_u32TimeoutCounter == SPI_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
        else
        {
          *RX_Data = SPI_Channel[ChannelConfig->Channel]->DR;
        }
        SPI_u8State = IDLE;
      }
    }
  }
  return Local_u8ErrorState;
}
