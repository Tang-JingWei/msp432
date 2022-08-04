#ifndef BSP_UART
#define BSP_UART

#include "sysinit.h"

typedef enum{
    UART0,
    UART1,
    UART2,
    UART3
}UART_CHA_enum;//串口可选通道枚举


void UART_send_Byte(UART_CHA_enum UART_CHA, uint8_t Data);
uint8_t UART_recv_Byte(UART_CHA_enum UART_CHA);
void UART_send_string(UART_CHA_enum UART_CHA, char *txt);
void UART_send_short(UART_CHA_enum UART_CHA, uint16_t num);
void UART_send_int(UART_CHA_enum UART_CHA, uint32_t num);



#endif // !BSP_UART
