#include "bsp_uart.h"

/*************************************************
 * 函  数  名:UART_send_Byte
 * 功      能:发送一个字节数据
 * 参      数:UART_CHA:UART可选通道，在bsp_uart.h中列出
 *          Data:要发送的8位数据
 * 注意事项:无
 *************************************************/
void UART_send_Byte(UART_CHA_enum UART_CHA, uint8_t Data)
{
  switch (UART_CHA)
  {
  case (UART0):
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG))
      ; //等待上一次发送完成
    EUSCI_A0->TXBUF = Data;
    break;
  case (UART1):
    while (!(EUSCI_A1->IFG & EUSCI_A_IFG_TXIFG))
      ; //等待上一次发送完成
    EUSCI_A1->TXBUF = Data;
    break;
  case (UART2):
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG))
      ; //等待上一次发送完成
    EUSCI_A2->TXBUF = Data;
    break;
  case (UART3):
    while (!(EUSCI_A3->IFG & EUSCI_A_IFG_TXIFG))
      ; //等待上一次发送完成
    EUSCI_A3->TXBUF = Data;
    break;
  default:;
  }
}
/*************************************************
 * 函  数  名:UART_recv_Byte
 * 功      能:接收一个字节数据
 * 参      数:UART_CHA:UART可选通道，在bsp_uart.h中列出
 * 注意事项:无
 *************************************************/
uint8_t UART_recv_Byte(UART_CHA_enum UART_CHA)
{
  uint8_t result;
  switch (UART_CHA)
  {
  case (UART0):
    result = EUSCI_A0->RXBUF;
    break; //取出缓冲区数据
  case (UART1):
    result = EUSCI_A1->RXBUF;
    break; //取出缓冲区数据
  case (UART2):
    result = EUSCI_A2->RXBUF;
    break; //取出缓冲区数据
  case (UART3):
    result = EUSCI_A3->RXBUF;
    break; //取出缓冲区数据
  default:;
  }
  return result;
}
/*************************************************
 * 函  数  名:UART_send_string
 * 功       能:发送一个字符串
 * 参       数:UART_CHA:UART可选通道，在bsp_uart.h中列出
 *          txt:所要发送的字符串
 * 注意事项:无
 *************************************************/
void UART_send_string(UART_CHA_enum UART_CHA, char *txt)
{
  int i;
  for (i = 0; txt[i]; i++)
  {
    UART_send_Byte(UART_CHA, txt[i]);
  }
}
/*************************************************
 * 函  数  名:UART_send_short
 * 功       能:发送一个16位整型
 * 参       数:UART_CHA:UART可选通道，在bsp_uart.h中列出
 *          num:所要发送的16位整型变量
 * 注意事项:从高位开始发送
 *************************************************/
void UART_send_short(UART_CHA_enum UART_CHA, uint16_t num)
{
  int i;
  for (i = 0; i < 2; i++)
  {
    UART_send_Byte(UART_CHA, (num & 0xff00) >> 8);
    num = num << 8;
  }
}
/*************************************************
 * 函  数  名:UART_send_int
 * 功       能:发送一个32位整型
 * 参       数:UART_CHA:UART可选通道，在bsp_uart.h中列出
 *          num:所要发送的32位整型变量
 * 注意事项:从高位开始发送
 *************************************************/
void UART_send_int(UART_CHA_enum UART_CHA, uint32_t num)
{
  int i;
  for (i = 0; i < 4; i++)
  {
    UART_send_Byte(UART_CHA, (num & 0xff000000) >> 24);
    num = num << 8;
  }
}
