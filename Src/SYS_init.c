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

void SYS_init(void)
{
  /* Enable CLK for Nedded peripheral */
  RCC_enumABPPerSts(RCC_APB2, RCC_SYSCFGEN, RCC_PER_ON);
  RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOBEN, RCC_PER_ON);
  RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOCEN, RCC_PER_ON);

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
}
