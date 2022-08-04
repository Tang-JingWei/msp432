#ifndef __USART_3_H
#define __USART_3_H

#include "sysinit.h"

#ifdef __TI_COMPILER_VERSION__
//CCSƽ̨
#include "stdarg.h"
#include "string.h"
#define USART0_MAX_SEND_LEN     600                 //����ͻ����ֽ���
int printf(const char *str, ...);
#endif

/*  Э�鶨���������೤�ȡ��������ݳ���(�ֽ�)��������������  */
#define LCD_MAX_LENGTH       4
#define LCD_MAX_DATA_BYTE    4
#define LCD_ASCII 					 1

/* ����3��ʼ�� */
void uart_3_init(uint32_t baudRate);

/* LCD���ݴ��� */
void Lcd_Data_Handle(void);

/* �������ݸ�LCD */
void SendDataToLcd(char *format, ...);


#endif

