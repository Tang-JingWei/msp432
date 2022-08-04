#ifndef __USART_2_H
#define __USART_2_H
#include "sysinit.h"

#ifdef __TI_COMPILER_VERSION__
//CCSƽ̨
#include "stdarg.h"
#include "string.h"
#define USART0_MAX_SEND_LEN     600                 //����ͻ����ֽ���
int printf(const char *str, ...);
#endif

/*  Э�鶨���������೤�ȡ��������ݳ���(�ֽ�)��������������  */
#define OPENMV_MAX_LENGTH       4
#define OPENMV_MAX_DATA_BYTE    4
#define OPENMV_ASCII 				    1

/* ����2��ʼ�� */
void uart_2_init(uint32_t baudRate);

/* Openmv���ݴ��� */
void Openmv_Data_Handle(void);

/* �������ݸ�Openmv */
void SendDataToOpenmv(char *format, ...);


#endif

