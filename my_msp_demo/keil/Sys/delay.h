/*******************************************
//MSP432P401R
//4 移植滴答延时

*******************************************/

#ifndef __DELAY_H
#define __DELAY_H

#include "sysinit.h"

void delay_init(void);
void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);

#endif
