/****************** UART_0 说明   *******************
 *
 * 上位机 占用 UART_0
 *  波特率：115200
 *  RX --- P1.2
 *  TX --- P1.3
 *
 ****************************************************/

#include "uart_0.h"

uint8_t Computer_Rxstate = 0;                      //接收状态
uint8_t Computer_Rxcount = 0;                      //接收次数
uint8_t Computer_Rxbuffer = 0;                     //接收缓存
uint16_t Computer_Rxbuff[200] = {0};               //缓冲区
uint16_t Computer_Data[COMPUTER_MAX_LENGTH] = {0}; //解析出的真实数据

#ifdef __TI_COMPILER_VERSION__
// CCS平台
uint8_t USART0_TX_BUF[USART0_MAX_SEND_LEN]; //发送缓冲,最大USART3_MAX_SEND_LEN字节
int printf(const char *str, ...)
{
  uint16_t i, j;
  va_list ap;
  va_start(ap, str);
  vsprintf((char *)USART0_TX_BUF, str, ap);
  va_end(ap);
  i = strlen((const char *)USART0_TX_BUF); //此次发送数据的长度
  for (j = 0; j < i; j++)                  //循环发送数据
  {
    // while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕
    UART_transmitData(EUSCI_A0_BASE, USART0_TX_BUF[j]);
  }
  return 0;
}
/*****************   函数说明   *****************
 *
 * 函数：int printf(const char *str, ...);
 * 源码来自@正点原子
 * 稍作改动适配CCS工程，在此也表感谢正点原子。
 *
 *****************   说明结束   *****************/

#else
// Keil支持标准C库跟微库
//预编译
// if 1 使用标准C库 如果报错就使用微库
// if 0 使用微库 得去勾选魔术棒里的 Use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
  int handle;
};
FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
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
/*****************   函数说明   *****************
 *
 * 以上两条对接标准输入输出库的函数:
 * int fputc(int ch, FILE *f);
 * int fgetc(FILE *f);
 * 源码为BiliBili平台UP主 “CloudBoyStudio” 编写
 * 本人RNA，不是作者
 * 在此也表感谢
 *
 *****************   说明结束   *****************/
#endif

void uart_0_init(uint32_t baudRate)
{
  /*___默认SMCLK 48MHz___ + ___比特率 115200___   */
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
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
                                              GPIO_PIN2 | GPIO_PIN3,
                                              GPIO_PRIMARY_MODULE_FUNCTION);

  /*  初始化串口  */
  UART_initModule(EUSCI_A0_BASE, &uartConfig);

  /*  开启串口模块  */
  UART_enableModule(EUSCI_A0_BASE);

  /*  开启串口相关中断 (EUSCI_A_UART_RECEIVE_INTERRUPT___接收中断___)  */
  UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

  /*  开启串口端口中断  */
  Interrupt_enableInterrupt(INT_EUSCIA0);

  /*  开启总中断  */
  Interrupt_enableMaster();

  /*  interrupt.c找中断服务函数  */
}

void EUSCIA0_IRQHandler()
{
  uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

  /*  判断是不是接收中断 */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    static uint16_t Data_temp = 0;  //暂时存放计算数据
    static uint8_t Split_count = 0; //分隔的每个数据单元计数

    /*  与stm32不同，每个字符需要用函数UART_receiveData手动接收  */
    Computer_Rxbuffer = UART_receiveData(EUSCI_A0_BASE);

    if (Computer_Rxstate == 0 && Computer_Rxbuffer == '#') //帧头
    {
      Computer_Rxstate = 1; //帧开始
      memset(Computer_Rxbuff, 0, sizeof(200));
    }
    else if (Computer_Rxstate == 1 && Computer_Rxbuffer == '$') //帧尾
    {
      if (Computer_Rxcount == (COMPUTER_MAX_LENGTH * COMPUTER_MAX_DATA_BYTE)) //接收完成(MAX_LENGTH * MAX_DATA_BYTE)位数据
      {
        for (int i = 0; i < COMPUTER_MAX_LENGTH; i++)
        {
          for (int j = 0; j < COMPUTER_MAX_DATA_BYTE; j++)
          {
            // printf("%c\r\n",Computer_Rxbuff[i*4+j]); //测试
#if (COMPUTER_ASCII == 1) //字符传输
            Data_temp += (Computer_Rxbuff[i * COMPUTER_MAX_DATA_BYTE + j] - '0') * pow(10, (COMPUTER_MAX_DATA_BYTE - 1 - j));
#else //数据传输
            Data_temp += (Computer_Rxbuff[i * COMPUTER_MAX_DATA_BYTE + j]) * pow(10, (COMPUTER_MAX_DATA_BYTE - 1 - j));
#endif
          }
          Computer_Data[i] = Data_temp;
          Data_temp = 0;
        }
        Computer_Data_Handle();
        //printf("%d %d %d %d\r\n",Computer_Data[0],Computer_Data[1],Computer_Data[2],Computer_Data[3]); //测试数据
        Computer_Rxstate = 0; //帧结束
        Computer_Rxcount = 0;
        Computer_Rxbuffer = 0;
        memset(Computer_Rxbuff, 0, sizeof(200));
      }
      else
      {
        //非正确协议
      }
    }
    else if (Computer_Rxstate == 1 && Computer_Rxbuffer == ',') //‘&’是协议的连接符，无意义，算法处理时跳过
    {
      if (Split_count != COMPUTER_MAX_DATA_BYTE) //每个数据都要是MAX_DATA_BYTE位才正确
      {
        //非正确协议
        Computer_Rxstate = 0;
        Computer_Rxcount = 0;
        Computer_Rxbuffer = 0;
        memset(Computer_Rxbuff, 0, sizeof(200));
      }
      Split_count = 0; //重置计数
    }
    else if (Computer_Rxstate == 1) // Rxstate == 1表示正在接收数据
    {
      Computer_Rxbuff[Computer_Rxcount++] = Computer_Rxbuffer;
      Split_count++;
    }

    /*  保险起见，最后清除中断标志位  */
    UART_clearInterruptFlag(EUSCI_A0_BASE, status);
  }
}

/* 上位机处理接收数据 */
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

/* 发送数据给上位机 */
void SendDataToComputer()
{
  /* 已经重定向到C库函数 printf */
}
