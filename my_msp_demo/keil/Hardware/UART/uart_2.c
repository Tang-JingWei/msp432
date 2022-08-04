/****************** UART_2 ˵��   *******************
 *
 * Openmv ռ�� UART_2
 *  �����ʣ�115200
 *  RX --- P3.2
 *  TX --- P3.3
 *
 ****************************************************/

#include "uart_2.h"

uint8_t Openmv_Rxstate = 0;							  //����״̬
uint8_t Openmv_Rxcount = 0;							  //���մ���
uint8_t Openmv_Rxbuffer = 0;							  //���ջ���
uint16_t Openmv_Rxbuff[200] = {0};   			  //������
uint16_t Openmv_Data[OPENMV_MAX_LENGTH] = {0};	//����������ʵ����

void uart_2_init(uint32_t baudRate)
{
  /*___Ĭ��SMCLK 48MHz___ + ___������ 9600___   */
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
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
                                              GPIO_PIN2 | GPIO_PIN3,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  ��ʼ������  */
  UART_initModule(EUSCI_A2_BASE, &uartConfig);

  /*  ��������ģ��  */
  UART_enableModule(EUSCI_A2_BASE);

  /*  ������������ж� (EUSCI_A_UART_RECEIVE_INTERRUPT___�����ж�___)  */
  UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  �������ڶ˿��ж�  */
  Interrupt_enableInterrupt(INT_EUSCIA2);

  /*  �������ж�  */
  // Interrupt_enableMaster();

  /*  interrupt.c���жϷ�����  */
}


/*
 *
 *  EUSCIA3 �жϷ����� ------- ������
 *
 */
void EUSCIA2_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

  /*  �ж��ǲ��ǽ����ж� */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint16_t Data_temp = 0;  //��ʱ��ż�������
    static uint8_t Split_count = 0; //�ָ���ÿ�����ݵ�Ԫ����

    /*  ��stm32��ͬ��ÿ���ַ���Ҫ�ú���UART_receiveData�ֶ�����  */
    Openmv_Rxbuffer = UART_receiveData(EUSCI_A2_BASE);

    if (Openmv_Rxstate == 0 && Openmv_Rxbuffer == '#') //֡ͷ
    {
      Openmv_Rxstate = 1; //֡��ʼ
      memset(Openmv_Rxbuff, 0, sizeof(200));
    }
    else if (Openmv_Rxstate == 1 && Openmv_Rxbuffer == '$') //֡β
    {
      if (Openmv_Rxcount == (BLE_MAX_LENGTH * BLE_MAX_DATA_BYTE)) //�������(MAX_LENGTH * MAX_DATA_BYTE)λ����
      {
        for (int i = 0; i < BLE_MAX_LENGTH; i++)
        {
          for (int j = 0; j < BLE_MAX_DATA_BYTE; j++)
          {
#if (OPENMV_ASCII == 1) //�ַ�����
            Data_temp += (Openmv_Rxbuff[i * BLE_MAX_DATA_BYTE + j] - '0') * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#else //���ݴ���
            Data_temp += (Openmv_Rxbuff[i * BLE_MAX_DATA_BYTE + j]) * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Openmv_Data[i] = Data_temp;
          Data_temp = 0;
        }
        //printf("%d %d %d %d\r\n", Data[0], Data[1], Data[2], Data[3]); //��������
        Openmv_Rxstate = 0;                                                   //֡����
        Openmv_Rxcount = 0;
        Openmv_Rxbuffer = 0;
        memset(Openmv_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //����ȷЭ��
      }
    }
    else if (Openmv_Rxstate == 1 && Openmv_Rxbuffer == ',') //��&����Э������ӷ��������壬�㷨����ʱ����
    {
      if (Split_count != BLE_MAX_DATA_BYTE) //ÿ�����ݶ�Ҫ��MAX_DATA_BYTEλ����ȷ
      {
        //����ȷЭ��
        Openmv_Rxstate = 0;
        Openmv_Rxcount = 0;
        Openmv_Rxbuffer = 0;
        memset(Openmv_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //���ü���
    }
    else if (Openmv_Rxstate == 1) // Rxstate == 1��ʾ���ڽ�������
    {
      Openmv_Rxbuff[Openmv_Rxcount++] = Openmv_Rxbuffer;
      Split_count++;
    }

    /*  ����������������жϱ�־λ  */
    UART_clearInterruptFlag(EUSCI_A2_BASE, status);
  }
}


/* �������ݸ����� */
void SendDataToOpenmv(char *format, ...)
{
  /* �ֽڻ����� ���ô��˾ͻῨ�� ��֪��ԭ�� ������Ϊ50 */
  char sendBuff[50];

  uint16_t i, j;
  va_list ap;
  va_start(ap, format);
  vsprintf((char *)sendBuff, format, ap);
  va_end(ap);

  UART_send_string(UART2, sendBuff);
	
}
