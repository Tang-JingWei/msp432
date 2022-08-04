/****************************************************/
// MSP432P401R
// ��������
/****************************************************/

#ifndef __UART_0_H
#define __UART_0_H

#include "sysinit.h"

#ifdef __TI_COMPILER_VERSION__
//CCSƽ̨
#include "stdarg.h"
#include "string.h"
#define USART0_MAX_SEND_LEN     600                 //����ͻ����ֽ���
int printf(const char *str, ...);
#endif

/*  Э�鶨���������೤�ȡ��������ݳ���(�ֽ�)��������������  */
#define COMPUTER_MAX_LENGTH       4
#define COMPUTER_MAX_DATA_BYTE    4
#define COMPUTER_ASCII 					  1

/* ����0��ʼ�� */
void uart_0_init(uint32_t baudRate);

/* �������ݴ��� */
void Computer_Data_Handle(void);

/* �������ݸ����� */
void SendDataToComputer(void);


#endif
