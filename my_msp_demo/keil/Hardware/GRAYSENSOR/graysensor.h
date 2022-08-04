#ifndef GRAYSENSOR
#define GRAYSENSOR


#include "sysinit.h"


#define GRAY_L2_GPIO_Port    GPIO_PORT_P10
#define GRAY_L1_GPIO_Port    GPIO_PORT_P10
#define GRAY_MID_GPIO_Port   GPIO_PORT_P10
#define GRAY_R1_GPIO_Port    GPIO_PORT_P10
#define GRAY_R2_GPIO_Port    GPIO_PORT_P10

#define GRAY_L2_Pin          GPIO_PIN4
#define GRAY_L1_Pin          GPIO_PIN4
#define GRAY_MID_Pin         GPIO_PIN4
#define GRAY_R1_Pin          GPIO_PIN4
#define GRAY_R2_Pin          GPIO_PIN4


/*  传感器标志位  */
extern uint8_t L2_flag, L1_flag, MID_flag, R1_flag, R2_flag;

/*  巡线补偿变量  */
extern float Search_Buchang;



void Gray_Init(void);
void Gray_Search(void);


#endif // !GRAYSENSOR

