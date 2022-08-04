#include "led.h"

//note：蓝灯取消（串口1占用）


void LED_Init(void)
{
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1);

    LED_RED_Off();
    LED_R_Off();
    LED_G_Off();
    // LED_B_Off();
}

void LED_RED_On(void) { LED_RED = 1; }
void LED_RED_Off(void) { LED_RED = 0; }
void LED_RED_Tog(void) { LED_RED ^= 1; }

void LED_R_Off(void) { LED_R = 0;}
void LED_G_Off(void) { LED_G = 0;}
// void LED_B_Off(void) { LED_B = 0; }

void LED_R_On(void) { LED_R = 1; }
void LED_G_On(void) { LED_G = 1;  }
// void LED_B_On(void) { LED_B = 1;  }

void LED_R_Tog(void) { LED_R ^= 1; }
void LED_G_Tog(void) { LED_G ^= 1; }
// void LED_B_Tog(void) { LED_B ^= 1; }


