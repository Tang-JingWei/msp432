/****************************************************/
// MSP432P401R
// ��������
/****************************************************/

#ifndef __USART_1_H
#define __USART_1_H

#include "sysinit.h"
#include "string.h"
#include "math.h"

/*  Э�鶨���������೤�ȡ��������ݳ���(�ֽ�)��������������  */
#define BLE_MAX_LENGTH       4
#define BLE_MAX_DATA_BYTE    4
#define BLE_ASCII 					 1

/* ����1��ʼ�� */
void uart_1_init(uint32_t baudRate);

/* �������ݴ��� */
void Ble_Data_Handle(void);

/* �������ݸ����� */
void SendDataToBle(char *format, ...);

#endif

