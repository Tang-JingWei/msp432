/******************  UART_1 ˵��   ******************
 *
 * Ble���� ռ�� UART_1
 *  ������  9600
 *  RX --- P2.2
 *  TX --- P2.3
 *
 ****************************************************/

#include "uart_1.h"

uint8_t Ble_Rxstate = 0;							  //����״̬
uint8_t Ble_Rxcount = 0;							  //���մ���
uint8_t Ble_Rxbuffer = 0;							  //���ջ���
uint16_t Ble_Rxbuff[200] = {0};   			  //������
uint16_t Ble_Data[BLE_MAX_LENGTH] = {0};	//����������ʵ����

void uart_1_init(uint32_t baudRate)
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
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
                                              GPIO_PIN2 | GPIO_PIN3,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  ��ʼ������  */
  UART_initModule(EUSCI_A1_BASE, &uartConfig);

  /*  ��������ģ��  */
  UART_enableModule(EUSCI_A1_BASE);

  /*  ������������ж� (EUSCI_A_UART_RECEIVE_INTERRUPT___�����ж�___)  */
  UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  �������ڶ˿��ж�  */
  Interrupt_enableInterrupt(INT_EUSCIA1);

  /*  �������ж�  */
  // Interrupt_enableMaster();
}

/*
*
*  EUSCIA1 �жϷ����� ------- Ble����
*
* @description: ���ڽ����жϺ���
  Э��˵����
  <1>
    ��λ-----------------��λ
      #xxxx,xxxx,xxxx,xxxx,$
    ֡��-----------------֡β
  <2>
    xxxx:  x��ʾһ���ֽڳ��ȵ����ݣ�xxxx��ʾ�����ĸ��ֽ�
     ��,��:  ��ʾ�ָ���(���Զ���)��Split_count���м������ж�ÿ�������Ƿ���ȷ��
           �������Ϊ ����,��Ϊֹ��ǰһ����������Ϊ���󣬸�֡���������ñ����������¸�֡
  <3>
    �㷨ԭ��1.�����ַ�ƴ�� --> �ַ�ת���֣�ASCII == 1��
             2.ֱ�Ӵ������ݣ���ȻҲҪ�涨��������ݳ��Ⱥ�������ٻ��зָ�����ASCII == 0��
    �㷨�ο���http://www.51hei.com/bbs/dpj-207742-1.html
* @param {UART_HandleTypeDef} *huart
* @author: �ƾ�ΰ
*
*
*/
void EUSCIA1_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

  /*  �ж��ǲ��ǽ����ж� */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint16_t Data_temp = 0;  //��ʱ��ż�������
    static uint8_t Split_count = 0; //�ָ���ÿ�����ݵ�Ԫ����

    /*  ��stm32��ͬ��ÿ���ַ���Ҫ�ú���UART_receiveData�ֶ�����  */
    Ble_Rxbuffer = UART_receiveData(EUSCI_A1_BASE);

    if (Ble_Rxstate == 0 && Ble_Rxbuffer == '#') //֡ͷ
    {
      Ble_Rxstate = 1; //֡��ʼ
      memset(Ble_Rxbuff, 0, sizeof(200));
    }
    else if (Ble_Rxstate == 1 && Ble_Rxbuffer == '$') //֡β
    {
      if (Ble_Rxcount == (BLE_MAX_LENGTH * BLE_MAX_DATA_BYTE)) //�������(MAX_LENGTH * MAX_DATA_BYTE)λ����
      {
        for (int i = 0; i < BLE_MAX_LENGTH; i++)
        {
          for (int j = 0; j < BLE_MAX_DATA_BYTE; j++)
          {
#if (BLE_ASCII == 1) //�ַ�����
            Data_temp += (Ble_Rxbuff[i * BLE_MAX_DATA_BYTE + j] - '0') * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#else           //���ݴ���
            Data_temp += (Ble_Rxbuff[i * BLE_MAX_DATA_BYTE + j]) * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Ble_Data[i] = Data_temp;
          Data_temp = 0;
        }
        //printf("%d %d %d %d\r\n",Ble_Data[0],Ble_Data[1],Ble_Data[2],Ble_Data[3]); //��������
        Ble_Rxstate = 0;                      //֡����
        Ble_Rxcount = 0;
        Ble_Rxbuffer = 0;
        memset(Ble_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //����ȷЭ��
      }
    }
    else if (Ble_Rxstate == 1 && Ble_Rxbuffer == ',') //��&����Э������ӷ��������壬�㷨����ʱ����
    {
      if (Split_count != BLE_MAX_DATA_BYTE) //ÿ�����ݶ�Ҫ��MAX_DATA_BYTEλ����ȷ
      {
        //����ȷЭ��
        Ble_Rxstate = 0;
        Ble_Rxcount = 0;
        Ble_Rxbuffer = 0;
        memset(Ble_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //���ü���
    }
    else if (Ble_Rxstate == 1) // Rxstate == 1��ʾ���ڽ�������
    {
      Ble_Rxbuff[Ble_Rxcount++] = Ble_Rxbuffer;
      Split_count++;
    }

    /*  ����������������жϱ�־λ  */
    UART_clearInterruptFlag(EUSCI_A1_BASE, status);
  }
}


/* ��������������� */
void Ble_Data_Handle()
{


}

/* �������ݸ����� */
void SendDataToBle(char *format, ...)
{
  /* �ֽڻ����� ���ô��˾ͻῨ�� ��֪��ԭ�� ������Ϊ50 */
  char sendBuff[50];

  uint16_t i, j;
  va_list ap;
  va_start(ap, format);
  vsprintf((char *)sendBuff, format, ap);
  va_end(ap);

  UART_send_string(UART1, sendBuff);
	
}
