/*
 * IR_Program.c
 *
 *  Created on: Aug 6, 2025
 *      Author: abdallah-shehawey
 */
#include <stdint.h>
#include "SYSTIC_interface.h"

#include "IR_interface.h"
#include "EXTI_interface.h"

extern EXTI_LineConfig_t NEXT_Line;
extern EXTI_LineConfig_t PAUSE_Line;
extern EXTI_LineConfig_t PREVIOUS_Line;

uint8_t Global_u8StartFlag = 0, Global_u8Counter = 0, Global_u8IRVal = 0xff;
uint32_t Global_u32IRDataArr[33] = {0};


void GET_vIRVal()
{
  if (Global_u8StartFlag == 0)
  {
    Global_u8StartFlag = 1;
    SYSTIC_enumCallbackSingleShot(Systic_func, 15000);
  }
  else
  {
    SYSTIC_enumGetElapsedTickSingleShot(&Global_u32IRDataArr[Global_u8Counter]);
    Global_u8Counter++;
    SYSTIC_enumCallbackSingleShot(Systic_func, 4000);
  }
}

void Systic_func(void)
{
  Global_u8StartFlag = 0;
  Global_u8Counter = 0;
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if ((Global_u32IRDataArr[17+i] <= 1250) && (Global_u32IRDataArr[17+i] >= 1000))
    {
      Global_u8IRVal &= ~(1 << i);
    }
    else if ((Global_u32IRDataArr[17+i] <= 2400) && (Global_u32IRDataArr[17+i] >= 2000))
    {
      Global_u8IRVal |= (1 << i);
    }
    else
    {

    }
  }
  if (Global_u8IRVal == 64)
  {
    EXTI_vSwIntEvent(&NEXT_Line);
    Global_u8IRVal = 0xff;
  }
  else if (Global_u8IRVal == 67)
  {
    EXTI_vSwIntEvent(&PAUSE_Line);
    Global_u8IRVal = 0xff;
  }
  else if (Global_u8IRVal == 68)
  {
    EXTI_vSwIntEvent(&PREVIOUS_Line);
    Global_u8IRVal = 0xff;
  }
  else
  {

  }
}
