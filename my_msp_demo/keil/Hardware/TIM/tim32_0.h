/****************************************************/
// MSP432P401R
// ¶¨Ê±Æ÷32_0
/****************************************************/

#ifndef __RNA_TIM32_0_H
#define __RNA_TIM32_0_H

#include "sysinit.h"

extern uint8_t timer_second;

void Tim32_0_Int_Init(uint32_t aar, uint8_t psc);

#endif
