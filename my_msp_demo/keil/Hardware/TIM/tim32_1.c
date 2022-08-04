/*********************************************************************************************************/

/**************************************        tim32_1          *******************************************/

#include "tim32_1.h"

/*  TIM32_0 Periodic modeģʽ  */
void Tim32_1_Int_Init(uint32_t aar, uint8_t psc)
{
    /* ��ʼ��timer32 ��ʱ��Դֻ����MCLK��pscֻ�� 1 16 256  */
    Timer32_initModule(TIMER32_1_BASE, psc, TIMER32_32BIT, TIMER32_PERIODIC_MODE);

    Timer32_setCount(TIMER32_1_BASE, aar);
    Timer32_enableInterrupt(TIMER32_1_BASE);
    Timer32_startTimer(TIMER32_1_BASE, false); //��������ģʽ--false ����ģʽ--true

    Interrupt_enableInterrupt(INT_T32_INT2);
}

/* Timer32_1 ISR */
void T32_INT2_IRQHandler(void)
{
    /*  �������ж�  */
    Timer32_clearInterruptFlag(TIMER32_1_BASE);

    /*��ʼ����û�����*/

    /*��������û�����*/
}
