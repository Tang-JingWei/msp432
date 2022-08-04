/****************** UART_0 ˵��   *******************
 *
 * ��λ�� ռ�� UART_0
 *  �����ʣ�115200
 *  RX --- P1.2
 *  TX --- P1.3
 *
 ****************************************************/

#include "uart_0.h"

uint8_t Computer_Rxstate = 0;                      //����״̬
uint8_t Computer_Rxcount = 0;                      //���մ���
uint8_t Computer_Rxbuffer = 0;                     //���ջ���
uint16_t Computer_Rxbuff[200] = {0};               //������
uint16_t Computer_Data[COMPUTER_MAX_LENGTH] = {0}; //����������ʵ����

#ifdef __TI_COMPILER_VERSION__
// CCSƽ̨
uint8_t USART0_TX_BUF[USART0_MAX_SEND_LEN]; //���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
int printf(const char *str, ...)
{
  uint16_t i, j;
  va_list ap;
  va_start(ap, str);
  vsprintf((char *)USART0_TX_BUF, str, ap);
  va_end(ap);
  i = strlen((const char *)USART0_TX_BUF); //�˴η������ݵĳ���
  for (j = 0; j < i; j++)                  //ѭ����������
  {
    // while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������
    UART_transmitData(EUSCI_A0_BASE, USART0_TX_BUF[j]);
  }
  return 0;
}
/*****************   ����˵��   *****************
 *
 * ������int printf(const char *str, ...);
 * Դ������@����ԭ��
 * �����Ķ�����CCS���̣��ڴ�Ҳ���л����ԭ�ӡ�
 *
 *****************   ˵������   *****************/

#else
// Keil֧�ֱ�׼C���΢��
//Ԥ����
// if 1 ʹ�ñ�׼C�� ��������ʹ��΢��
// if 0 ʹ��΢�� ��ȥ��ѡħ������� Use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
  int handle;
};
FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
  x = x;
}
#else
int fgetc(FILE *f)
{
  while (EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG !=
         UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG))
    ;
  return UART_receiveData(EUSCI_A0_BASE);
}
#endif
int fputc(int ch, FILE *f)
{
  UART_transmitData(EUSCI_A0_BASE, ch & 0xFF);
  return ch;
}
/*****************   ����˵��   *****************
 *
 * ���������Խӱ�׼���������ĺ���:
 * int fputc(int ch, FILE *f);
 * int fgetc(FILE *f);
 * Դ��ΪBiliBiliƽ̨UP�� ��CloudBoyStudio�� ��д
 * ����RNA����������
 * �ڴ�Ҳ���л
 *
 *****************   ˵������   *****************/
#endif

void uart_0_init(uint32_t baudRate)
{
  /*___Ĭ��SMCLK 48MHz___ + ___������ 115200___   */
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
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
                                              GPIO_PIN2 | GPIO_PIN3,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  ��ʼ������  */
  UART_initModule(EUSCI_A0_BASE, &uartConfig);

  /*  ��������ģ��  */
  UART_enableModule(EUSCI_A0_BASE);

  /*  ������������ж� (EUSCI_A_UART_RECEIVE_INTERRUPT___�����ж�___)  */
  UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  �������ڶ˿��ж�  */
  Interrupt_enableInterrupt(INT_EUSCIA0);

  /*  �������ж�  */
  Interrupt_enableMaster();

  /*  interrupt.c���жϷ�����  */
}

void EUSCIA0_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

  /*  �ж��ǲ��ǽ����ж� */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint16_t Data_temp = 0;  //��ʱ��ż�������
    static uint8_t Split_count = 0; //�ָ���ÿ�����ݵ�Ԫ����

    /*  ��stm32��ͬ��ÿ���ַ���Ҫ�ú���UART_receiveData�ֶ�����  */
    Computer_Rxbuffer = UART_receiveData(EUSCI_A0_BASE);

    if (Computer_Rxstate == 0 && Computer_Rxbuffer == '#') //֡ͷ
    {
      Computer_Rxstate = 1; //֡��ʼ
      memset(Computer_Rxbuff, 0, sizeof(200));
    }
    else if (Computer_Rxstate == 1 && Computer_Rxbuffer == '$') //֡β
    {
      if (Computer_Rxcount == (COMPUTER_MAX_LENGTH * COMPUTER_MAX_DATA_BYTE)) //�������(MAX_LENGTH * MAX_DATA_BYTE)λ����
      {
        for (int i = 0; i < COMPUTER_MAX_LENGTH; i++)
        {
          for (int j = 0; j < COMPUTER_MAX_DATA_BYTE; j++)
          {
            // printf("%c\r\n",Computer_Rxbuff[i*4+j]); //����
#if (COMPUTER_ASCII == 1) //�ַ�����
            Data_temp += (Computer_Rxbuff[i * COMPUTER_MAX_DATA_BYTE + j] - '0') * pow(10, (COMPUTER_MAX_DATA_BYTE - 1 - j));
#else //���ݴ���
            Data_temp += (Computer_Rxbuff[i * COMPUTER_MAX_DATA_BYTE + j]) * pow(10, (COMPUTER_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Computer_Data[i] = Data_temp;
          Data_temp = 0;
        }
        Computer_Data_Handle();
        //printf("%d %d %d %d\r\n",Computer_Data[0],Computer_Data[1],Computer_Data[2],Computer_Data[3]); //��������
        Computer_Rxstate = 0; //֡����
        Computer_Rxcount = 0;
        Computer_Rxbuffer = 0;
        memset(Computer_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //����ȷЭ��
      }
    }
    else if (Computer_Rxstate == 1 && Computer_Rxbuffer == ',') //��&����Э������ӷ��������壬�㷨����ʱ����
    {
      if (Split_count != COMPUTER_MAX_DATA_BYTE) //ÿ�����ݶ�Ҫ��MAX_DATA_BYTEλ����ȷ
      {
        //����ȷЭ��
        Computer_Rxstate = 0;
        Computer_Rxcount = 0;
        Computer_Rxbuffer = 0;
        memset(Computer_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //���ü���
    }
    else if (Computer_Rxstate == 1) // Rxstate == 1��ʾ���ڽ�������
    {
      Computer_Rxbuff[Computer_Rxcount++] = Computer_Rxbuffer;
      Split_count++;
    }

    /*  ����������������жϱ�־λ  */
    UART_clearInterruptFlag(EUSCI_A0_BASE, status);
  }
}

/* ��λ������������� */
void Computer_Data_Handle()
{
  if(Computer_Data[0] == 1)
  {
    // MPU6050_Init(); 
    // 
  }
  LED_G_Tog();
  SendDataToLcd("setting.t2.txt=\"%d\"",Computer_Data[1]);
}

/* �������ݸ���λ�� */
void SendDataToComputer()
{
  /* �Ѿ��ض���C�⺯�� printf */
}
