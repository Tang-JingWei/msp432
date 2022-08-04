/****************************************************/
// MSP432P401R
// 串口配置
/****************************************************/

#ifndef __USART_1_H
#define __USART_1_H

#include "sysinit.h"
#include "string.h"
#include "math.h"

/*  协议定义数据种类长度、单个数据长度(字节)、传输数据类型  */
#define BLE_MAX_LENGTH       4
#define BLE_MAX_DATA_BYTE    4
#define BLE_ASCII 					 1

/* 串口1初始化 */
void uart_1_init(uint32_t baudRate);

/* 蓝牙数据处理 */
void Ble_Data_Handle(void);

/* 发送数据给蓝牙 */
void SendDataToBle(char *format, ...);

#endif

