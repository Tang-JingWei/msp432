/****************************************************/
// MSP432P401R
// ADC14采集
/****************************************************/

/*********************
 *
 * 最大采集电压 3.3V
 *
 * ADC采集引脚：
 * 单路 为 P5.5
 * 双路 为 P5.5 P5.4
 * 三路 为 P5.5 P5.4 P5.3
 *
 ************************/

/*

笔记：

  1、多个通道序列转换模式，只要使能最后通道的中断，因为中断触发是靠ADC_MEMx来触发的
     API：
     For example, when the ADC_MEM0 location finishes a conversion cycle,
     the ADC_INT0 interrupt will be set.
  2、ADC_MEMx 和 ADC_INPUT_Ax(对应了引脚的) 不是一定要一一对应的。
     例如：ADC_MEM0 可以存任意 INPUT_Ax口 来的的数据
     MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, false);
  3、定时器定时采样时，下面的方法是采用定时器的比较功能，设置了up mode之后(CCR0)，再设置比较结构体。
     触发的点是：达到compare配置的CCR的值。那么既然配置了compare，如果结构体写了TIMER_A_OUTPUTMODE_SET_RESET，
     并且对应GPIO设置了复用，那么这个引脚就可以产生PWM波

*/

#include "bsp_adc14.h"

// #define ADC_TimerTrigger_mode 1

// #define ADC_TimerTrigger_Timer 
// #define ADC_TimerTrigge_INPUT ADC_INPUT_A1
// #define ADC_TimerTrigger_GPIO GPIO_PORT_P5
// #define ADC_TimerTrigger_Pin GPIO_PIN4
// #define ADC_TimerTrigge_CLK
// #define ADC_TimerTrigge_Period 1000
// #define ADC_TimerTrigge_Pulse  500

//总时间  M*N*21us
#define N 200 //采样次数
#define M 2   //采样通道个数

static uint16_t resultsBuffer[M];

/*  ADC 配置 */
void ADC_Config(void)
{
  /* 启用浮点运算的FPU */
  MAP_FPU_enableModule();
  MAP_FPU_enableLazyStacking();

  /* Initializing ADC (MCLK/4/5) */
  MAP_ADC14_enableModule();                                                                 //使能ADC14模块
  MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_5, ADC_NOROUTE); //初始化ADC 时钟 分频  通道 2.4MHz

#if M == 1
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION); //模拟输入
  MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);                                                    //单通道配置 多次转化true
  MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);       //使用内部电源电压参考 非差分输入false
  MAP_ADC14_enableInterrupt(ADC_INT0);                                                                    // ADC通道0的中断

#elif M == 2
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
  MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true); //多通道配置 多次转化true
  MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A0, false);
  MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A1, false);
  MAP_ADC14_enableInterrupt(ADC_INT1);

#elif M == 3
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION); //
  MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
  MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A0, false);
  MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A1, false);
  MAP_ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A2, false);
  MAP_ADC14_enableInterrupt(ADC_INT2);

#endif
  /* Enabling Interrupts */
  MAP_Interrupt_enableInterrupt(INT_ADC14); // ADC模块的中断

  /* Setting up the sample timer to automatically step through the sequence
   * convert.
   */
  MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION); //自动触发

  /* Triggering the start of the sample */
  MAP_ADC14_enableConversion();        //使能开始转换(触发后 自动ADC上电)
  MAP_ADC14_toggleConversionTrigger(); //开启第一次软件触发
}

/*  定时器定时触发ADC 配置 */
void ADC_TimerTrigger_Config()
{
  /* 启用浮点运算的FPU */
  MAP_FPU_enableModule();
  MAP_FPU_enableLazyStacking();

  /* Initializing ADC (MCLK/4/5) */
  MAP_ADC14_enableModule();
  MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_5, 0);

  /* Configuring GPIOs (5.4 A1) */
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
  //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

  /* Configuring ADC Memory */
  MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true); //多次转化-true
  MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, false);

  /* Timer_A Up Mode Configuration Parameter */
  Timer_A_UpModeConfig upModeConfig =
      {
          TIMER_A_CLOCKSOURCE_ACLK,            // Clock Source
          TIMER_A_CLOCKSOURCE_DIVIDER_1,       // Divide
          1000,                                // Period
          TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
          TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
          TIMER_A_DO_CLEAR                     // Clear Counter
      };
  /* Configuring Timer_A0 in up mode and sourced from SMCLK */
  Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);

  Timer_A_CompareModeConfig compareConfig =
      {
          TIMER_A_CAPTURECOMPARE_REGISTER_1,        // Use CCR1
          TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable CCR interrupt
          TIMER_A_OUTPUTMODE_SET_RESET,             // Toggle output but
          160                                       // Pulse
      };
  /* Configuring Timer_A0 in CCR1 to trigger */
  Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);

  /* Configuring the sample trigger to be sourced from Timer_A0  and setting it
   * to automatic iteration after it is triggered*/
  MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

  /* Enabling the interrupt when a conversion on channel 1 is complete and
   * enabling conversions */
  MAP_ADC14_enableInterrupt(ADC_INT0);
  MAP_ADC14_enableConversion();

  /* Enabling Interrupts */
  MAP_Interrupt_enableInterrupt(INT_ADC14);
  MAP_Interrupt_enableMaster();

  /* Starting the Timer */
  MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

/*
 *  ADC14 中断服务函数
 *
 *
 */
void ADC14_IRQHandler(void)
{
  uint8_t i = 0;
  uint_fast64_t status = MAP_ADC14_getEnabledInterruptStatus();
  MAP_ADC14_clearInterruptFlag(status);

  #if M == 1
    if (ADC_INT0 & status)
  #elif M == 2
    if (ADC_INT1 & status)
  #elif M == 3
    if (ADC_INT2 & status)
  #endif
    {
      MAP_ADC14_getMultiSequenceResult(resultsBuffer);
      for (i = 0; i < M; i++)
        printf("[%d]:%d\r\t", i, resultsBuffer[i]);
      printf("\r\n");
    }

  if (ADC_INT0 & status)
  {
    printf("[%.2f] \r\n", (float)MAP_ADC14_getResult(ADC_MEM0) * 3.3 / 16384.0);
  }
}
