/****************** UART_3 ˵��   *******************
 *
 *  ���������� ռ�� UART_3
 *  �����ʣ� ��115200��
 *  RX --- P9.6
 *  TX --- P9.7
 *
 ****************************************************/

#include "uart_3.h"

uint8_t Lcd_Rxstate = 0;                    //����״̬
uint8_t Lcd_Rxcount = 0;                    //���մ���
uint8_t Lcd_Rxbuffer = 0;                   //���ջ���
uint16_t Lcd_Rxbuff[200] = {0};             //������
uint16_t Lcd_Data[OPENMV_MAX_LENGTH] = {0}; //����������ʵ����

void uart_3_init(uint32_t baudRate)
{
  /*___Ĭ��SMCLK 48MHz___ + ___������ ()___   */
  const eUSCI_UART_ConfigV1 uartConfig =
      {
          EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // SMCLK Clock Source
          26,                                            // BRDIV = 26
          0,                                             // UCxBRF = 0
          111,                                           // UCxBRS = 111
          EUSCI_A_UART_NO_PARITY,                        // No Parity
          EUSCI_A_UART_LSB_FIRST,                        // MSB First
          EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
          EUSCI_A_UART_MODE,                             // UART mode
          EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
          EUSCI_A_UART_8_BIT_LEN                         // 8 bit data length
      };

  /*   ���ò�����  */
  eusci_calcBaudDividers((eUSCI_UART_ConfigV1 *)&uartConfig, baudRate);

  /*  ѡ���������ó�UARTģʽ��GPIO���ã�  */
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P9,
                                              GPIO_PIN6 | GPIO_PIN7,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  ��ʼ������  */
  UART_initModule(EUSCI_A3_BASE, &uartConfig);

  /*  ��������ģ��  */
  UART_enableModule(EUSCI_A3_BASE);

  /*  ������������ж� (EUSCI_A_UART_RECEIVE_INTERRUPT___�����ж�___)  */
  UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  �������ڶ˿��ж�  */
  Interrupt_enableInterrupt(INT_EUSCIA3);

  /*  �������ж�  */
  // Interrupt_enableMaster();
}

/*
 *
 *  EUSCIA3 �жϷ����� ------- ������
 *
 */
void EUSCIA3_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A3_BASE);

  /*  �ж��ǲ��ǽ����ж� */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint32_t Data_temp = 0;  //��ʱ��ż�������
    static uint8_t Split_count = 0; //�ָ���ÿ�����ݵ�Ԫ����

    /*  ��stm32��ͬ��ÿ���ַ���Ҫ�ú���UART_receiveData�ֶ�����  */
    Lcd_Rxbuffer = UART_receiveData(EUSCI_A3_BASE);

    if (Lcd_Rxstate == 0 && Lcd_Rxbuffer == '#') //֡ͷ
    {
      Lcd_Rxstate = 1; //֡��ʼ
      memset(Lcd_Rxbuff, 0, sizeof(200));
    }
    else if (Lcd_Rxstate == 1 && Lcd_Rxbuffer == '$') //֡β
    {
      if (Lcd_Rxcount == (LCD_MAX_LENGTH * LCD_MAX_DATA_BYTE)) //�������(MAX_LENGTH * MAX_DATA_BYTE)λ����
      {
        for (int i = 0; i < LCD_MAX_LENGTH; i++)
        {
          for (int j = 0; j < LCD_MAX_DATA_BYTE; j++)
          {
#if (LCD_ASCII == 1) //�ַ�����
            Data_temp += (Lcd_Rxbuff[i * LCD_MAX_DATA_BYTE + j] - '0') * pow(10, (LCD_MAX_DATA_BYTE - 1 - j));
#else //���ݴ���
            Data_temp += (Lcd_Rxbuff[i * LCD_MAX_DATA_BYTE + j]) * pow(10, (LCD_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Lcd_Data[i] = Data_temp;
          Data_temp = 0;
        }
        // printf("%d %d %d %d\r\n", Data[0], Data[1], Data[2], Data[3]); //��������
        Lcd_Rxstate = 0; //֡����
        Lcd_Rxcount = 0;
        Lcd_Rxbuffer = 0;
        memset(Lcd_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //����ȷЭ��
      }
    }
    else if (Lcd_Rxstate == 1 && Lcd_Rxbuffer == ',') //��&����Э������ӷ��������壬�㷨����ʱ����
    {
      if (Split_count != LCD_MAX_DATA_BYTE) //ÿ�����ݶ�Ҫ��MAX_DATA_BYTEλ����ȷ
      {
        //����ȷЭ��
        Lcd_Rxstate = 0;
        Lcd_Rxcount = 0;
        Lcd_Rxbuffer = 0;
        memset(Lcd_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //���ü���
    }
    else if (Lcd_Rxstate == 1) // Rxstate == 1��ʾ���ڽ�������
    {
      Lcd_Rxbuff[Lcd_Rxcount++] = Lcd_Rxbuffer;
      Split_count++;
    }

    /*  ����������������жϱ�־λ  */
    UART_clearInterruptFlag(EUSCI_A3_BASE, status);
  }
}

/*
 *   �������ݸ� ������
 *
 */
void SendDataToLcd(char *format, ...)
{
  /*
     �ֽڻ����� ���ô��˾ͻῨ�� ��֪��ԭ��
     �ѽ����ԭ���� stack_size ̫С�������������������ļ��и���
  */
  char sendBuff[100];

  va_list ap;
  va_start(ap, format);
  vsprintf((char *)sendBuff, format, ap);
  va_end(ap);

  UART_send_string(UART3, sendBuff);
  UART_send_string(UART3, "\xff\xff\xff"); /* HIM ��������� */
}
