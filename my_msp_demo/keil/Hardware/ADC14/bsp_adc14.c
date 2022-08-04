/****************************************************/
// MSP432P401R
// ADC14�ɼ�
/****************************************************/

/*********************
 *
 * ���ɼ���ѹ 3.3V
 *
 * ADC�ɼ����ţ�
 * ��· Ϊ P5.5
 * ˫· Ϊ P5.5 P5.4
 * ��· Ϊ P5.5 P5.4 P5.3
 *
 ************************/

/*

�ʼǣ�

  1�����ͨ������ת��ģʽ��ֻҪʹ�����ͨ�����жϣ���Ϊ�жϴ����ǿ�ADC_MEMx��������
     API��
     For example, when the ADC_MEM0 location finishes a conversion cycle,
     the ADC_INT0 interrupt will be set.
  2��ADC_MEMx �� ADC_INPUT_Ax(��Ӧ�����ŵ�) ����һ��Ҫһһ��Ӧ�ġ�
     ���磺ADC_MEM0 ���Դ����� INPUT_Ax�� ���ĵ�����
     MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, false);
  3����ʱ����ʱ����ʱ������ķ����ǲ��ö�ʱ���ıȽϹ��ܣ�������up mode֮��(CCR0)�������ñȽϽṹ�塣
     �����ĵ��ǣ��ﵽcompare���õ�CCR��ֵ����ô��Ȼ������compare������ṹ��д��TIMER_A_OUTPUTMODE_SET_RESET��
     ���Ҷ�ӦGPIO�����˸��ã���ô������žͿ��Բ���PWM��

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

//��ʱ��  M*N*21us
#define N 200 //��������
#define M 2   //����ͨ������

static uint16_t resultsBuffer[M];

/*  ADC ���� */
void ADC_Config(void)
{
  /* ���ø��������FPU */
  MAP_FPU_enableModule();
  MAP_FPU_enableLazyStacking();

  /* Initializing ADC (MCLK/4/5) */
  MAP_ADC14_enableModule();                                                                 //ʹ��ADC14ģ��
  MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_5, ADC_NOROUTE); //��ʼ��ADC ʱ�� ��Ƶ  ͨ�� 2.4MHz

#if M == 1
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION); //ģ������
  MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);                                                    //��ͨ������ ���ת��true
  MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);       //ʹ���ڲ���Դ��ѹ�ο� �ǲ������false
  MAP_ADC14_enableInterrupt(ADC_INT0);                                                                    // ADCͨ��0���ж�

#elif M == 2
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
  MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true); //��ͨ������ ���ת��true
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
  MAP_Interrupt_enableInterrupt(INT_ADC14); // ADCģ����ж�

  /* Setting up the sample timer to automatically step through the sequence
   * convert.
   */
  MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION); //�Զ�����

  /* Triggering the start of the sample */
  MAP_ADC14_enableConversion();        //ʹ�ܿ�ʼת��(������ �Զ�ADC�ϵ�)
  MAP_ADC14_toggleConversionTrigger(); //������һ���������
}

/*  ��ʱ����ʱ����ADC ���� */
void ADC_TimerTrigger_Config()
{
  /* ���ø��������FPU */
  MAP_FPU_enableModule();
  MAP_FPU_enableLazyStacking();

  /* Initializing ADC (MCLK/4/5) */
  MAP_ADC14_enableModule();
  MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_5, 0);

  /* Configuring GPIOs (5.4 A1) */
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
  //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

  /* Configuring ADC Memory */
  MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true); //���ת��-true
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
 *  ADC14 �жϷ�����
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
