/*
 * SYS_init.c
 *
 *  Created on: Aug 5, 2025
 *      Author: abdallah-shehawey
 */

#include <stdint.h>
#include <string.h>
#include "STM32F446xx.h"
#include "STD_MACROS.h"
#include "ErrTypes.h"

#include "RCC_interface.h"
#include "GPIO_interface.h"
#include "SYSTIC_interface.h"
#include "NVIC_interface.h"
#include "EXTI_interface.h"
#include "USART_intreface.h"
#include "SYSCFG_interface.h"

#include "SYS_init.h"
#include "IR_interface.h"

/*************************************************************/
/* Configure Output Pin for DAC (PORTC) */
GPIO_8PinsConfig_t PORTC =
{
    .StartPin = GPIO_PIN0,
    .Port   = GPIO_PORTC,
    .Mode   = GPIO_OUTPUT,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_MEDIUM_SPEED,
};
/*************************************************************/
/* Configre INT Lines and it's PIN */
GPIO_PinConfig_t SW_NEXT =
{
    GPIO_PORTC,
    GPIO_PIN10,
    GPIO_INPUT,
    .PullType = GPIO_PULL_UP,
};

GPIO_PinConfig_t SW_PAUSE =
{
    GPIO_PORTC,
    GPIO_PIN11,
    GPIO_INPUT,
    .PullType = GPIO_PULL_UP,
};

GPIO_PinConfig_t SW_PREVIOUS =
{
    GPIO_PORTC,
    GPIO_PIN12,
    GPIO_INPUT,
    .PullType = GPIO_PULL_UP,
};



EXTI_LineConfig_t NEXT_Line =
{
    EXTI_LINE10,
    EXTI_FALLING_EDGE,
    EXTI_EN,
    NEXT,
};

EXTI_LineConfig_t PAUSE_Line =
{
    EXTI_LINE11,
    EXTI_FALLING_EDGE,
    EXTI_EN,
    PAUSE_RESUME,
};

EXTI_LineConfig_t PREVIOUS_Line =
{
    EXTI_LINE12,
    EXTI_FALLING_EDGE,
    EXTI_EN,
    PREVIOUS,
};
/*************************************************************/
GPIO_PinConfig_t IR_PIN =
{
    GPIO_PORTC,
    GPIO_PIN9,
    GPIO_INPUT,
    .PullType = GPIO_PULL_UP,
};

EXTI_LineConfig_t IR_LINE =
{
    EXTI_LINE9,
    EXTI_FALLING_EDGE,
    EXTI_EN,
    GET_vIRVal,
};
/*************************************************************/
USART_Config_t USART_3 =
{
    .BaudRate = 9600,
    .Channel  = USART_CHANNEL3,
    .HardwareFlowControl = UART_HWCONTROL_NONE,
    .Mode = USART_MODE_TX,
    .OverSampling = USART_OVERSAMPLING_8,
    .Parity = USART_PARITY_NONE,
    .StopBits = USART_STOPBITS_1,
    .WordLength = USART_WORDLENGTH_8B,
};

GPIO_PinConfig_t USART3_TX =
{
    .Port     = GPIO_PORTB,
    .PinNum = GPIO_PIN10,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_HIGH_SPEED,
    .AlternateFunction = GPIO_AF7
};

GPIO_PinConfig_t USART3_RX =
{
    .Port     = GPIO_PORTB,
    .PinNum = GPIO_PIN11,
    .Mode   = GPIO_ALTFN,
    .Otype  = GPIO_PUSH_PULL,
    .PullType = GPIO_NO_PULL,
    .Speed    = GPIO_HIGH_SPEED,
    .AlternateFunction = GPIO_AF7
};

void SYS_init(void)
{
  /* Enable CLK for Nedded peripheral */
  RCC_enumABPPerSts(RCC_APB2, RCC_SYSCFGEN, RCC_PER_ON);
  RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOBEN, RCC_PER_ON);
  RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOCEN, RCC_PER_ON);
  RCC_enumABPPerSts(RCC_APB1, RCC_USART3EN, RCC_PER_ON);
  RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOAEN, RCC_PER_ON);

  /* Enable INT and it's line and ISR for Button in the System */
  /* Enable 3 INT */

  /* configure it's buttons*/
  GPIO_enumPinInit(&SW_NEXT);
  GPIO_enumPinInit(&SW_PAUSE);
  GPIO_enumPinInit(&SW_PREVIOUS);
  /* Enable 3 INT */
  SYSCFG_vSetEXTIConfig(SYSCFG_EXTI10, SYSCFG_PORTC);
  SYSCFG_vSetEXTIConfig(SYSCFG_EXTI11, SYSCFG_PORTC);
  SYSCFG_vSetEXTIConfig(SYSCFG_EXTI12, SYSCFG_PORTC);
  /* enable for NVIC */
  NVIC_vEnableIRQ(NVIC_EXTI15_10);
  /* Enable EXT INT */
  EXTI_vLineInit(&NEXT_Line);
  EXTI_vLineInit(&PAUSE_Line);
  EXTI_vLineInit(&PREVIOUS_Line);

  GPIO_enumPinInit(&IR_PIN);
  SYSCFG_vSetEXTIConfig(SYSCFG_EXTI9, SYSCFG_PORTC);
  NVIC_vEnableIRQ(NVIC_EXTI9_5);
  EXTI_vLineInit(&IR_LINE);

  SYSTIC_vInit();
  GPIO_enumPort8PinsInit(&PORTC);

  GPIO_enumPinInit(&USART3_TX);
  GPIO_enumPinInit(&USART3_RX);
  USART_Init(&USART_3);
}
