/****************** UART_3 说明   *******************
 *
 *  （串口屏） 占用 UART_3
 *  波特率： （115200）
 *  RX --- P9.6
 *  TX --- P9.7
 *
 ****************************************************/

#include "uart_3.h"

uint8_t Lcd_Rxstate = 0;                    //接收状态
uint8_t Lcd_Rxcount = 0;                    //接收次数
uint8_t Lcd_Rxbuffer = 0;                   //接收缓存
uint16_t Lcd_Rxbuff[200] = {0};             //缓冲区
uint16_t Lcd_Data[OPENMV_MAX_LENGTH] = {0}; //解析出的真实数据

void uart_3_init(uint32_t baudRate)
{
  /*___默认SMCLK 48MHz___ + ___比特率 ()___   */
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

  /*   配置波特率  */
  eusci_calcBaudDividers((eUSCI_UART_ConfigV1 *)&uartConfig, baudRate);

  /*  选择引脚配置成UART模式（GPIO复用）  */
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P9,
                                              GPIO_PIN6 | GPIO_PIN7,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  初始化串口  */
  UART_initModule(EUSCI_A3_BASE, &uartConfig);

  /*  开启串口模块  */
  UART_enableModule(EUSCI_A3_BASE);

  /*  开启串口相关中断 (EUSCI_A_UART_RECEIVE_INTERRUPT___接收中断___)  */
  UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  开启串口端口中断  */
  Interrupt_enableInterrupt(INT_EUSCIA3);

  /*  开启总中断  */
  // Interrupt_enableMaster();
}

/*
 *
 *  EUSCIA3 中断服务函数 ------- 串口屏
 *
 */
void EUSCIA3_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A3_BASE);

  /*  判断是不是接收中断 */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint32_t Data_temp = 0;  //暂时存放计算数据
    static uint8_t Split_count = 0; //分隔的每个数据单元计数

    /*  与stm32不同，每个字符需要用函数UART_receiveData手动接收  */
    Lcd_Rxbuffer = UART_receiveData(EUSCI_A3_BASE);

    if (Lcd_Rxstate == 0 && Lcd_Rxbuffer == '#') //帧头
    {
      Lcd_Rxstate = 1; //帧开始
      memset(Lcd_Rxbuff, 0, sizeof(200));
    }
    else if (Lcd_Rxstate == 1 && Lcd_Rxbuffer == '$') //帧尾
    {
      if (Lcd_Rxcount == (LCD_MAX_LENGTH * LCD_MAX_DATA_BYTE)) //接收完成(MAX_LENGTH * MAX_DATA_BYTE)位数据
      {
        for (int i = 0; i < LCD_MAX_LENGTH; i++)
        {
          for (int j = 0; j < LCD_MAX_DATA_BYTE; j++)
          {
#if (LCD_ASCII == 1) //字符传输
            Data_temp += (Lcd_Rxbuff[i * LCD_MAX_DATA_BYTE + j] - '0') * pow(10, (LCD_MAX_DATA_BYTE - 1 - j));
#else //数据传输
            Data_temp += (Lcd_Rxbuff[i * LCD_MAX_DATA_BYTE + j]) * pow(10, (LCD_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Lcd_Data[i] = Data_temp;
          Data_temp = 0;
        }
        // printf("%d %d %d %d\r\n", Data[0], Data[1], Data[2], Data[3]); //测试数据
        Lcd_Rxstate = 0; //帧结束
        Lcd_Rxcount = 0;
        Lcd_Rxbuffer = 0;
        memset(Lcd_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //非正确协议
      }
    }
    else if (Lcd_Rxstate == 1 && Lcd_Rxbuffer == ',') //‘&’是协议的连接符，无意义，算法处理时跳过
    {
      if (Split_count != LCD_MAX_DATA_BYTE) //每个数据都要是MAX_DATA_BYTE位才正确
      {
        //非正确协议
        Lcd_Rxstate = 0;
        Lcd_Rxcount = 0;
        Lcd_Rxbuffer = 0;
        memset(Lcd_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //重置计数
    }
    else if (Lcd_Rxstate == 1) // Rxstate == 1表示正在接收数据
    {
      Lcd_Rxbuff[Lcd_Rxcount++] = Lcd_Rxbuffer;
      Split_count++;
    }

    /*  保险起见，最后清除中断标志位  */
    UART_clearInterruptFlag(EUSCI_A3_BASE, status);
  }
}

/*
 *   发送数据给 串口屏
 *
 */
void SendDataToLcd(char *format, ...)
{
  /*
     字节缓冲区 设置大了就会卡死 不知道原因
     已解决：原因是 stack_size 太小，导致死机，在启动文件中更改
  */
  char sendBuff[100];

  va_list ap;
  va_start(ap, format);
  vsprintf((char *)sendBuff, format, ap);
  va_end(ap);

  UART_send_string(UART3, sendBuff);
  UART_send_string(UART3, "\xff\xff\xff"); /* HIM 命令结束符 */
}
