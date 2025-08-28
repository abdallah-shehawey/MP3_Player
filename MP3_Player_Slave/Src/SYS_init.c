/*
 * SYS_init.c
 *
 *  Created on: Aug 7, 2025
 *      Author: abdallah-shehawey
 */
#include <stdint.h>

#include "STM32F446xx.h"
#include "ErrTypes.h"
#include "STD_MACROS.h"

#include "SYSTIC_interface.h"
#include "GPIO_interface.h"
#include "RCC_interface.h"
#include "SYSCFG_interface.h"
#include "SPI_interface.h"
#include "USART_intreface.h"
#include "NVIC_interface.h"
#include "TFT_interface.h"

#include "SYS_init.h"

SPI_Config_t SPI1 =
{
    .Channel = SPI_CHANNEL1,
    .CPHA = SPI_CPHA_1EDGE,
    .CPOL = SPI_CPOL_LOW,
    .Mode = SPI_MODE_MASTER,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
    .SPE = SPI_SPE_EN,
    .BFIRST = SPI_MSBFIRST,
    .NSS_MAN = SPI_NSS_SOFTWARE,
    .NSSI_MODE = SPI_NSSI_NOT_SELECT,
    .RXONLY = SPI_RXONLY_DISABLE,
    .DFF = SPI_DFF_8BIT,
    .CRC_MODE = SPI_CRCDIS,
    .DIMODE = SPI_UNIDIMODE,
    .BIDIOE = SPI_OUTPUT_EN,
};

GPIO_PinConfig_t PA4_NSS =
{
    .Port     = GPIO_PORTA,
    .PinNum = GPIO_PIN4,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_HIGH_SPEED,
    .AlternateFunction = GPIO_AF5
};

GPIO_PinConfig_t PA5_SCK =
{
    .Port     = GPIO_PORTA,
    .PinNum = GPIO_PIN5,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_MEDIUM_SPEED,
    .AlternateFunction = GPIO_AF5
};
GPIO_PinConfig_t PA6_MISO =
{
    .Port     = GPIO_PORTA,
    .PinNum = GPIO_PIN6,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_HIGH_SPEED,
    .AlternateFunction = GPIO_AF5
};

GPIO_PinConfig_t PA7_MOSI =
{
    .Port     = GPIO_PORTA,
    .PinNum = GPIO_PIN7,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_MEDIUM_SPEED,
    .AlternateFunction = GPIO_AF5
};

USART_Handle_t USART_2 =
{
    .BaudRate = 9600,
    .Channel  = USART_CHANNEL2,
    .HardwareFlowControl = UART_HWCONTROL_NONE,
    .Mode = USART_MODE_RX,
    .OverSampling = USART_OVERSAMPLING_8,
    .Parity = USART_PARITY_NONE,
    .StopBits = USART_STOPBITS_1,
    .WordLength = USART_WORDLENGTH_8B,
    .RXNEIE = USART_RXNEIE_EN,
    .TCIE = USART_TCIE_DIS,
    .TXEIE = USART_TXEIE_DIS,
    .IDLEIE = USART_IDLEIE_DIS,
    .PEIE = USART_PEIE_DIS,
    USART_RXCMP,
};

GPIO_PinConfig_t PA2 =
{
    .Port     = GPIO_PORTA,
    .PinNum = GPIO_PIN2,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_HIGH_SPEED,
    .AlternateFunction = GPIO_AF7
};

GPIO_PinConfig_t PA3 =
{
    .Port     = GPIO_PORTA,
    .PinNum = GPIO_PIN3,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_HIGH_SPEED,
    .AlternateFunction = GPIO_AF7
};


void SYS_vinit(void)
{
  SYSTIC_vInit();
  RCC_enumABPPerSts(RCC_APB2, RCC_SPI1EN, RCC_PER_ON);
  RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOAEN, RCC_PER_ON);
  GPIO_enumPinInit(&PA4_NSS);
  GPIO_enumPinInit(&PA5_SCK);
  GPIO_enumPinInit(&PA6_MISO);
  GPIO_enumPinInit(&PA7_MOSI);
  SPI_enumInit(&SPI1);
  HTFT_vinit(&SPI1);

  RCC_enumABPPerSts(RCC_APB1, RCC_USART2EN, RCC_PER_ON);
  GPIO_enumPinInit(&PA2);
  GPIO_enumPinInit(&PA3);
  NVIC_vEnableIRQ(NVIC_USART2);
  USART_InitIT(&USART_2);

}
