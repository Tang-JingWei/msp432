/****************** UART_2 说明   *******************
 *
 * Openmv 占用 UART_2
 *  波特率：115200
 *  RX --- P3.2
 *  TX --- P3.3
 *
 ****************************************************/

#include "uart_2.h"

uint8_t Openmv_Rxstate = 0;							  //接收状态
uint8_t Openmv_Rxcount = 0;							  //接收次数
uint8_t Openmv_Rxbuffer = 0;							  //接收缓存
uint16_t Openmv_Rxbuff[200] = {0};   			  //缓冲区
uint16_t Openmv_Data[OPENMV_MAX_LENGTH] = {0};	//解析出的真实数据

void uart_2_init(uint32_t baudRate)
{
  /*___默认SMCLK 48MHz___ + ___比特率 9600___   */
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
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
                                              GPIO_PIN2 | GPIO_PIN3,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  初始化串口  */
  UART_initModule(EUSCI_A2_BASE, &uartConfig);

  /*  开启串口模块  */
  UART_enableModule(EUSCI_A2_BASE);

  /*  开启串口相关中断 (EUSCI_A_UART_RECEIVE_INTERRUPT___接收中断___)  */
  UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  开启串口端口中断  */
  Interrupt_enableInterrupt(INT_EUSCIA2);

  /*  开启总中断  */
  // Interrupt_enableMaster();

  /*  interrupt.c找中断服务函数  */
}


/*
 *
 *  EUSCIA3 中断服务函数 ------- 串口屏
 *
 */
void EUSCIA2_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

  /*  判断是不是接收中断 */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint16_t Data_temp = 0;  //暂时存放计算数据
    static uint8_t Split_count = 0; //分隔的每个数据单元计数

    /*  与stm32不同，每个字符需要用函数UART_receiveData手动接收  */
    Openmv_Rxbuffer = UART_receiveData(EUSCI_A2_BASE);

    if (Openmv_Rxstate == 0 && Openmv_Rxbuffer == '#') //帧头
    {
      Openmv_Rxstate = 1; //帧开始
      memset(Openmv_Rxbuff, 0, sizeof(200));
    }
    else if (Openmv_Rxstate == 1 && Openmv_Rxbuffer == '$') //帧尾
    {
      if (Openmv_Rxcount == (BLE_MAX_LENGTH * BLE_MAX_DATA_BYTE)) //接收完成(MAX_LENGTH * MAX_DATA_BYTE)位数据
      {
        for (int i = 0; i < BLE_MAX_LENGTH; i++)
        {
          for (int j = 0; j < BLE_MAX_DATA_BYTE; j++)
          {
#if (OPENMV_ASCII == 1) //字符传输
            Data_temp += (Openmv_Rxbuff[i * BLE_MAX_DATA_BYTE + j] - '0') * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#else //数据传输
            Data_temp += (Openmv_Rxbuff[i * BLE_MAX_DATA_BYTE + j]) * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Openmv_Data[i] = Data_temp;
          Data_temp = 0;
        }
        //printf("%d %d %d %d\r\n", Data[0], Data[1], Data[2], Data[3]); //测试数据
        Openmv_Rxstate = 0;                                                   //帧结束
        Openmv_Rxcount = 0;
        Openmv_Rxbuffer = 0;
        memset(Openmv_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //非正确协议
      }
    }
    else if (Openmv_Rxstate == 1 && Openmv_Rxbuffer == ',') //‘&’是协议的连接符，无意义，算法处理时跳过
    {
      if (Split_count != BLE_MAX_DATA_BYTE) //每个数据都要是MAX_DATA_BYTE位才正确
      {
        //非正确协议
        Openmv_Rxstate = 0;
        Openmv_Rxcount = 0;
        Openmv_Rxbuffer = 0;
        memset(Openmv_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //重置计数
    }
    else if (Openmv_Rxstate == 1) // Rxstate == 1表示正在接收数据
    {
      Openmv_Rxbuff[Openmv_Rxcount++] = Openmv_Rxbuffer;
      Split_count++;
    }

    /*  保险起见，最后清除中断标志位  */
    UART_clearInterruptFlag(EUSCI_A2_BASE, status);
  }
}


/* 发送数据给蓝牙 */
void SendDataToOpenmv(char *format, ...)
{
  /* 字节缓冲区 设置大了就会卡死 不知道原因 现在设为50 */
  char sendBuff[50];

  uint16_t i, j;
  va_list ap;
  va_start(ap, format);
  vsprintf((char *)sendBuff, format, ap);
  va_end(ap);

  UART_send_string(UART2, sendBuff);
	
}
