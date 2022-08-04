/******************  UART_1 说明   ******************
 *
 * Ble蓝牙 占用 UART_1
 *  波特率  9600
 *  RX --- P2.2
 *  TX --- P2.3
 *
 ****************************************************/

#include "uart_1.h"

uint8_t Ble_Rxstate = 0;							  //接收状态
uint8_t Ble_Rxcount = 0;							  //接收次数
uint8_t Ble_Rxbuffer = 0;							  //接收缓存
uint16_t Ble_Rxbuff[200] = {0};   			  //缓冲区
uint16_t Ble_Data[BLE_MAX_LENGTH] = {0};	//解析出的真实数据

void uart_1_init(uint32_t baudRate)
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
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
                                              GPIO_PIN2 | GPIO_PIN3,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  初始化串口  */
  UART_initModule(EUSCI_A1_BASE, &uartConfig);

  /*  开启串口模块  */
  UART_enableModule(EUSCI_A1_BASE);

  /*  开启串口相关中断 (EUSCI_A_UART_RECEIVE_INTERRUPT___接收中断___)  */
  UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  开启串口端口中断  */
  Interrupt_enableInterrupt(INT_EUSCIA1);

  /*  开启总中断  */
  // Interrupt_enableMaster();
}

/*
*
*  EUSCIA1 中断服务函数 ------- Ble蓝牙
*
* @description: 串口接收中断函数
  协议说明：
  <1>
    高位-----------------低位
      #xxxx,xxxx,xxxx,xxxx,$
    帧首-----------------帧尾
  <2>
    xxxx:  x表示一个字节长度的数据，xxxx表示接收四个字节
     ‘,’:  表示分隔符(可自定义)，Split_count进行计数，判断每个数据是否正确，
           否则可认为 到‘,’为止的前一个接收数据为错误，该帧丢弃，重置变量，接收下个帧
  <3>
    算法原理：1.单个字符拼接 --> 字符转数字（ASCII == 1）
             2.直接传输数据，当然也要规定传输的数据长度和种类多少还有分隔符（ASCII == 0）
    算法参考：http://www.51hei.com/bbs/dpj-207742-1.html
* @param {UART_HandleTypeDef} *huart
* @author: 唐京伟
*
*
*/
void EUSCIA1_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

  /*  判断是不是接收中断 */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint16_t Data_temp = 0;  //暂时存放计算数据
    static uint8_t Split_count = 0; //分隔的每个数据单元计数

    /*  与stm32不同，每个字符需要用函数UART_receiveData手动接收  */
    Ble_Rxbuffer = UART_receiveData(EUSCI_A1_BASE);

    if (Ble_Rxstate == 0 && Ble_Rxbuffer == '#') //帧头
    {
      Ble_Rxstate = 1; //帧开始
      memset(Ble_Rxbuff, 0, sizeof(200));
    }
    else if (Ble_Rxstate == 1 && Ble_Rxbuffer == '$') //帧尾
    {
      if (Ble_Rxcount == (BLE_MAX_LENGTH * BLE_MAX_DATA_BYTE)) //接收完成(MAX_LENGTH * MAX_DATA_BYTE)位数据
      {
        for (int i = 0; i < BLE_MAX_LENGTH; i++)
        {
          for (int j = 0; j < BLE_MAX_DATA_BYTE; j++)
          {
#if (BLE_ASCII == 1) //字符传输
            Data_temp += (Ble_Rxbuff[i * BLE_MAX_DATA_BYTE + j] - '0') * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#else           //数据传输
            Data_temp += (Ble_Rxbuff[i * BLE_MAX_DATA_BYTE + j]) * pow(10, (BLE_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Ble_Data[i] = Data_temp;
          Data_temp = 0;
        }
        //printf("%d %d %d %d\r\n",Ble_Data[0],Ble_Data[1],Ble_Data[2],Ble_Data[3]); //测试数据
        Ble_Rxstate = 0;                      //帧结束
        Ble_Rxcount = 0;
        Ble_Rxbuffer = 0;
        memset(Ble_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //非正确协议
      }
    }
    else if (Ble_Rxstate == 1 && Ble_Rxbuffer == ',') //‘&’是协议的连接符，无意义，算法处理时跳过
    {
      if (Split_count != BLE_MAX_DATA_BYTE) //每个数据都要是MAX_DATA_BYTE位才正确
      {
        //非正确协议
        Ble_Rxstate = 0;
        Ble_Rxcount = 0;
        Ble_Rxbuffer = 0;
        memset(Ble_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //重置计数
    }
    else if (Ble_Rxstate == 1) // Rxstate == 1表示正在接收数据
    {
      Ble_Rxbuff[Ble_Rxcount++] = Ble_Rxbuffer;
      Split_count++;
    }

    /*  保险起见，最后清除中断标志位  */
    UART_clearInterruptFlag(EUSCI_A1_BASE, status);
  }
}


/* 蓝牙处理接收数据 */
void Ble_Data_Handle()
{


}

/* 发送数据给蓝牙 */
void SendDataToBle(char *format, ...)
{
  /* 字节缓冲区 设置大了就会卡死 不知道原因 现在设为50 */
  char sendBuff[50];

  uint16_t i, j;
  va_list ap;
  va_start(ap, format);
  vsprintf((char *)sendBuff, format, ap);
  va_end(ap);

  UART_send_string(UART1, sendBuff);
	
}
