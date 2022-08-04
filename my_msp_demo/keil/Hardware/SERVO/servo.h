#ifndef SERVO
#define SERVO

#include "sysinit.h"

/* ¶æ»ú´úºÅ */
typedef enum
{
	down_servo = 0,
	up_servo
}servo_code;



void Servo_Move(servo_code code, float angle);
void Servos_Relocate(void);
void Servos_Timer_Init(void);


#endif






