#include "sysinit.h"

#define SR04T_GPIO_Port   GPIO_PORT_P4
#define SR04T_Pin         GPIO_PIN5
#define SR04R_GPIO_Port   GPIO_PORT_P5
#define SR04R_Pin         GPIO_PIN6


/* ≤‚¡øæ‡¿Î */
extern float sr04_distance;


void UltrasonicWave_TimerInit(void);
void UltrasonicWave_StartMeasure(void);
float UltrasonicWave_Measure(void);



