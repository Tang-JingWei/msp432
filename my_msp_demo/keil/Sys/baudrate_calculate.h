/****************************************************/
// MSP432P401R
// 串口波特率计算
// Bilibili：m-RNA
// E-mail:m-RNA@qq.com
/****************************************************/
#ifndef __RNA_BAUDRATE_CALCULATE_H
#define __RNA_BAUDRATE_CALCULATE_H
#include "sysinit.h"

//错误指示灯宏定义 方便移植使用
//MSP432P401R 有两个红灯P1.0 P2.0
//#define WARN_LED_1_PORT GPIO_PORT_P1
//#define WARN_LED_2_PORT GPIO_PORT_P2
//#define WARN_LED_1_PIN GPIO_PIN0
//#define WARN_LED_2_PIN GPIO_PIN0
//#define WARN_LED_INIT MAP_GPIO_setAsOutputPin
//#define WARN_LED_ON MAP_GPIO_setOutputHighOnPin
//#define WARN_LED_OFF MAP_GPIO_setOutputLowOnPin

#ifdef EUSCI_A_UART_7_BIT_LEN
void eusci_calcBaudDividers(eUSCI_UART_ConfigV1 *uart_config, uint32_t baudRate); //固件库v3_40_01_02
#else
void eusci_calcBaudDividers(eUSCI_UART_Config *uart_config, uint32_t baudRate); //固件库v3_21_00_05
#endif

#endif
