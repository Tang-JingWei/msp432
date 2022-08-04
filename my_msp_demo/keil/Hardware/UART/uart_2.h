#ifndef __USART_2_H
#define __USART_2_H
#include "sysinit.h"

#ifdef __TI_COMPILER_VERSION__
//CCS平台
#include "stdarg.h"
#include "string.h"
#define USART0_MAX_SEND_LEN     600                 //最大发送缓存字节数
int printf(const char *str, ...);
#endif

/*  协议定义数据种类长度、单个数据长度(字节)、传输数据类型  */
#define OPENMV_MAX_LENGTH       4
#define OPENMV_MAX_DATA_BYTE    4
#define OPENMV_ASCII 				    1

/* 串口2初始化 */
void uart_2_init(uint32_t baudRate);

/* Openmv数据处理 */
void Openmv_Data_Handle(void);

/* 发送数据给Openmv */
void SendDataToOpenmv(char *format, ...);


#endif

