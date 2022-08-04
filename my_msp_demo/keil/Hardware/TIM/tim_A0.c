/*********************************************************************************************************/

/**************************************         TIMA0          *******************************************/
/*


笔记：
TAx有 CCR0 CCR1~CCR4(CCRn)，相当于有5个通道
  1、CCR0是专门存放Period值的寄存器,该比较捕获寄存器占用一个中断号（INT_TA0_0）和中断ISR（TA0_0_IRQHandler）；
  2、其他CCRn共占一个中断号（INT_TA0_N）和一个ISR（TA0_N_IRQHandler），用于PWM、捕获等，在中断函数中要判断是哪个通道；
  3、up mode的中断就是 TA0_0_IRQHandler ，前提是 captureCompareInterruptEnable_CCR0_CCIE 要使能。
  4、文档：
    Two interrupt vectors are associated with the 16-bit Timer_A module:
      ? TAxCCR0 interrupt vector for TAxCCR0 CCIFG
      ? TAxIV interrupt vector for all other CCIFG flags and TAIFG ---> ！！！溢出中断也是（INT_TA0_N）和（TA0_N_IRQHandler）
    论坛：
    You still have to write an interrupt function for CCR0 and/or CCR1,2,3,... 
    The CCR0 function has a dedicated one and it's flag is cleared when entering the ISR.
    CCR1,2,3,... and TAIFG share one interrupt function. 
  5、TAxR 是 Timer_A register. The TAxR register is the count of Timer_A. 用来存放count值的（目前计数值）
    也可以用 TIMER_A_CMSIS(TIMER_A0_BASE)->R 对TAxR 进行写和读操作：
       写-->TIMER_A_CMSIS(TIMER_A0_BASE)->R = 0;
       读-->temp = TIMER_A_CMSIS(TIMER_A0_BASE)->R;
    ①在 SR04 应用中可以 给该寄存器写或者读，以获得时间间隔
*/
#include "tim_A0.h"

/*  增计数模式  */
void TimA0_Int_Init(uint16_t ccr0, uint16_t psc)
{
    // 1.增计数模式初始化
    Timer_A_UpModeConfig upConfig;
    upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                      //时钟源
    upConfig.clockSourceDivider = psc;                                                     //时钟分频 范围1-64
    upConfig.timerPeriod = ccr0;                                                           //自动重装载值（ARR）
    upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                   //禁用 tim溢出中断
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //启用 ccr0更新中断
    upConfig.timerClear = TIMER_A_DO_CLEAR;                                                // Clear value

    // 2.初始化定时器A
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

    // 3.选择模式开始计数（初始化定时器后还要使用该函数启动）
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    // 4.清除比较中断标志位
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // 5.开启定时器端口中断
    Interrupt_enableInterrupt(INT_TA0_0);

		/*  6.开启总中断 (放在主函数一起开启就好了) */
//		MAP_Interrupt_enableMaster();
}



/*
*  TIMA0 中断服务函数
*  
*	
*/
void TA0_0_IRQHandler(void)
{
		/* 清除中断标志位 */
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /* 开始填充用户代码 */
    GetMotorPulse();
    
    /* 结束填充用户代码 */
}

void TA0_N_IRQHandler(void)
{
		/* 清除中断标志位 */
		MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    /* 开始填充用户代码 */
		
    /* 结束填充用户代码 */
}


