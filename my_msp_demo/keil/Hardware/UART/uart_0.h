/****************************************************/
// MSP432P401R
// 串口配置
/****************************************************/

#ifndef __UART_0_H
#define __UART_0_H

#include "sysinit.h"

#ifdef __TI_COMPILER_VERSION__
//CCS平台
#include "stdarg.h"
#include "string.h"
#define USART0_MAX_SEND_LEN     600                 //最大发送缓存字节数
int printf(const char *str, ...);
#endif

/*  协议定义数据种类长度、单个数据长度(字节)、传输数据类型  */
#define COMPUTER_MAX_LENGTH       4
#define COMPUTER_MAX_DATA_BYTE    4
#define COMPUTER_ASCII 					  1

/* 串口0初始化 */
void uart_0_init(uint32_t baudRate);

/* 蓝牙数据处理 */
void Computer_Data_Handle(void);

/* 发送数据给蓝牙 */
void SendDataToComputer(void);


#endif
