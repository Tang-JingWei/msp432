/*********************************************************************************************************/

/**************************************        tim32_1          *******************************************/

#include "tim32_1.h"

/*  TIM32_0 Periodic mode模式  */
void Tim32_1_Int_Init(uint32_t aar, uint8_t psc)
{
    /* 初始化timer32 ，时钟源只能是MCLK，psc只有 1 16 256  */
    Timer32_initModule(TIMER32_1_BASE, psc, TIMER32_32BIT, TIMER32_PERIODIC_MODE);

    Timer32_setCount(TIMER32_1_BASE, aar);
    Timer32_enableInterrupt(TIMER32_1_BASE);
    Timer32_startTimer(TIMER32_1_BASE, false); //连续计数模式--false 单次模式--true

    Interrupt_enableInterrupt(INT_T32_INT2);
}

/* Timer32_1 ISR */
void T32_INT2_IRQHandler(void)
{
    /*  清除溢出中断  */
    Timer32_clearInterruptFlag(TIMER32_1_BASE);

    /*开始填充用户代码*/

    /*结束填充用户代码*/
}
