/*********************************************************************************************************/

/**************************************         TIMA1          *******************************************/

#include "tim_A1.h"


#define CAP_TIMA_SELECTION 			 TIMER_A1_BASE                         //这里改定时器
#define CAP_REGISTER_SELECTION   TIMER_A_CAPTURECOMPARE_REGISTER_1     //这里改定时器通道
#define CAP_CCR_NUM 					   1                                     //
#define CAP_PORT_PIN             GPIO_PORT_P5, GPIO_PIN6               //这里改复用引脚


/*
*
相关函数： 
				MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, i);


*/


/*   timA1 PWM    */
void TimA1_PWM_Init(uint16_t psc, uint16_t ccr0, uint16_t pulse)
{
    /*  初始化引脚  */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

		/*  定时器PWM初始化  */
    Timer_A_PWMConfig TimA1_PWMConfig;
    TimA1_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;             //时钟源
    TimA1_PWMConfig.clockSourceDivider = psc;                            //时钟分频 范围1-64
    TimA1_PWMConfig.timerPeriod = ccr0;                                  //自动重装载值（ARR）
    TimA1_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //通道一 （引脚选择）
    TimA1_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;   //输出模式
    TimA1_PWMConfig.dutyCycle = pulse;                                   //占空比

		/*  初始化比较寄存器以产生 PWM1。
    API上说了使用这个函数就开启了定时器（Generate a PWM with timer running in up mode） */
    Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_PWMConfig); 
}
/*********************************************************************************************************/


/*
*  TIMA1 中断服务函数
*  
*	
*/
void TA1_0_IRQHandler(void)
{
		/* 清除中断标志位 */
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /* 开始填充用户代码 */
		

    /* 结束填充用户代码 */
}

void TA1_N_IRQHandler(void)
{
		/* 清除中断标志位 */
		MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    /* 开始填充用户代码 */
		
    
    /* 结束填充用户代码 */
}


