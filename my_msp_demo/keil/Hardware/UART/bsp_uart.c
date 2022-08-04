#include "bsp_uart.h"

/*************************************************
 * ��  ��  ��:UART_send_Byte
 * ��      ��:����һ���ֽ�����
 * ��      ��:UART_CHA:UART��ѡͨ������bsp_uart.h���г�
 *          Data:Ҫ���͵�8λ����
 * ע������:��
 *************************************************/
void UART_send_Byte(UART_CHA_enum UART_CHA, uint8_t Data)
{
  switch (UART_CHA)
  {
  case (UART0):
    while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG))
      ; //�ȴ���һ�η������
    EUSCI_A0->TXBUF = Data;
    break;
  case (UART1):
    while (!(EUSCI_A1->IFG & EUSCI_A_IFG_TXIFG))
      ; //�ȴ���һ�η������
    EUSCI_A1->TXBUF = Data;
    break;
  case (UART2):
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG))
      ; //�ȴ���һ�η������
    EUSCI_A2->TXBUF = Data;
    break;
  case (UART3):
    while (!(EUSCI_A3->IFG & EUSCI_A_IFG_TXIFG))
      ; //�ȴ���һ�η������
    EUSCI_A3->TXBUF = Data;
    break;
  default:;
  }
}
/*************************************************
 * ��  ��  ��:UART_recv_Byte
 * ��      ��:����һ���ֽ�����
 * ��      ��:UART_CHA:UART��ѡͨ������bsp_uart.h���г�
 * ע������:��
 *************************************************/
uint8_t UART_recv_Byte(UART_CHA_enum UART_CHA)
{
  uint8_t result;
  switch (UART_CHA)
  {
  case (UART0):
    result = EUSCI_A0->RXBUF;
    break; //ȡ������������
  case (UART1):
    result = EUSCI_A1->RXBUF;
    break; //ȡ������������
  case (UART2):
    result = EUSCI_A2->RXBUF;
    break; //ȡ������������
  case (UART3):
    result = EUSCI_A3->RXBUF;
    break; //ȡ������������
  default:;
  }
  return result;
}
/*************************************************
 * ��  ��  ��:UART_send_string
 * ��       ��:����һ���ַ���
 * ��       ��:UART_CHA:UART��ѡͨ������bsp_uart.h���г�
 *          txt:��Ҫ���͵��ַ���
 * ע������:��
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
 * ��  ��  ��:UART_send_short
 * ��       ��:����һ��16λ����
 * ��       ��:UART_CHA:UART��ѡͨ������bsp_uart.h���г�
 *          num:��Ҫ���͵�16λ���ͱ���
 * ע������:�Ӹ�λ��ʼ����
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
 * ��  ��  ��:UART_send_int
 * ��       ��:����һ��32λ����
 * ��       ��:UART_CHA:UART��ѡͨ������bsp_uart.h���г�
 *          num:��Ҫ���͵�32λ���ͱ���
 * ע������:�Ӹ�λ��ʼ����
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
