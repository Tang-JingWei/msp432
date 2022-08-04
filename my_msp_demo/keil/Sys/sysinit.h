#ifndef __SYSINIT_H__
#define __SYSINIT_H__


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/***宏 定 义****/
#define u8   uint8_t
#define u16  uint16_t
#define u32  uint32_t
/**************/

/***用 户 头 文 件***/
#include "led.h"
#include "key.h"
#include "delay.h"
#include "stdio.h" //1.61328125kb
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "uart_0.h"
#include "uart_1.h"
#include "uart_2.h"
#include "uart_3.h"
#include "bsp_uart.h"
#include "baudrate_calculate.h"
#include "bsp_math.h"
#include "tim_A0.h"
#include "tim_A1.h"
#include "tim_A2.h"
#include "tim_A3.h"
#include "tim32_0.h"
#include "tim32_1.h"
#include "oled.h"
#include "bsp_systick.h"
#include "sr04.h"
#include "mpu6050.h"
#include "myiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "servo.h"
#include "graysensor.h"
#include "motor.h"
/*********************/

extern float yaw;
extern float roll;
extern float pitch;

void SysInit(void);



#endif



