/*********************************************************************************************************/

/**************************************         TIMA3          *******************************************/

#include "tim_A3.h"

/*  比较模式的 PWM输出  */
void TimA3_Com_Init(uint16_t compare)
{
  /*  初始化引脚  */
  MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

  /* 定时器配置参数 */
  Timer_A_ContinuousModeConfig continuousModeConfig = {
      TIMER_A_CLOCKSOURCE_SMCLK,      // SMCLK Clock Source
      TIMER_A_CLOCKSOURCE_DIVIDER_48, // SMCLK/48 = 1MHz
      TIMER_A_TAIE_INTERRUPT_DISABLE, // 开启定时器溢出中断（溢出时TAIFG置位，进入TAx_N_IRQHander）
      TIMER_A_DO_CLEAR                // Clear Counter
  };
  /* 将定时器初始化为连续计数模式  */
  MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &continuousModeConfig);

  /*  配置比较模式  */
  Timer_A_CompareModeConfig compareConfig_PWM;
  compareConfig_PWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;               //通道一（引脚定义）
  compareConfig_PWM.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE; //关闭CCR中断
  compareConfig_PWM.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;                 //输出模式
  compareConfig_PWM.compareValue = compare;                                            //比较值
  /*  初始化定时器的Compare Mode */
  Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM);

  /*  开启定时器开始计数 */
  Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UPDOWN_MODE);

  /*  没有中断 ,因为结构体里面 disable 了 CCR中断和溢出中断  */
}



/*
 *  TIMA0 中断服务函数
 *
 *
 */
void TA3_0_IRQHandler(void)
{
  /* 清除中断标志位 */
  MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

  /* 开始填充用户代码 */

  /* 结束填充用户代码 */
}

void TA3_N_IRQHandler(void)
{
  /* 清除中断标志位 */
  MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

  /* 开始填充用户代码 */

  /* 结束填充用户代码 */
}
