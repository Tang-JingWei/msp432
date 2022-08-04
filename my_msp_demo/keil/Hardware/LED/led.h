#ifndef __LED_H__
#define __LED_H__


#include "sysinit.h"

// Î»´ø²Ù×÷
#define LED_RED  BITBAND_PERI(P1OUT,0)
#define LED_R 	 BITBAND_PERI(P2OUT,0)
#define LED_G 	 BITBAND_PERI(P2OUT,1)
// #define LED_B 	 BITBAND_PERI(P2OUT,2)

void LED_Init(void);

void LED_RED_On(void);
void LED_RED_Off(void);
void LED_RED_Tog(void);

void LED_R_On(void);
void LED_G_On(void);
// void LED_B_On(void);

void LED_R_Off(void);
void LED_G_Off(void);
// void LED_B_Off(void);

void LED_R_Tog(void);
void LED_G_Tog(void);
// void LED_B_Tog(void);


#endif



