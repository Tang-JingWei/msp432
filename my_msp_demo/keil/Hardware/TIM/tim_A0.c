/*********************************************************************************************************/

/**************************************         TIMA0          *******************************************/
/*


�ʼǣ�
TAx�� CCR0 CCR1~CCR4(CCRn)���൱����5��ͨ��
  1��CCR0��ר�Ŵ��Periodֵ�ļĴ���,�ñȽϲ���Ĵ���ռ��һ���жϺţ�INT_TA0_0�����ж�ISR��TA0_0_IRQHandler����
  2������CCRn��ռһ���жϺţ�INT_TA0_N����һ��ISR��TA0_N_IRQHandler��������PWM������ȣ����жϺ�����Ҫ�ж����ĸ�ͨ����
  3��up mode���жϾ��� TA0_0_IRQHandler ��ǰ���� captureCompareInterruptEnable_CCR0_CCIE Ҫʹ�ܡ�
  4���ĵ���
    Two interrupt vectors are associated with the 16-bit Timer_A module:
      ? TAxCCR0 interrupt vector for TAxCCR0 CCIFG
      ? TAxIV interrupt vector for all other CCIFG flags and TAIFG ---> ����������ж�Ҳ�ǣ�INT_TA0_N���ͣ�TA0_N_IRQHandler��
    ��̳��
    You still have to write an interrupt function for CCR0 and/or CCR1,2,3,... 
    The CCR0 function has a dedicated one and it's flag is cleared when entering the ISR.
    CCR1,2,3,... and TAIFG share one interrupt function. 
  5��TAxR �� Timer_A register. The TAxR register is the count of Timer_A. �������countֵ�ģ�Ŀǰ����ֵ��
    Ҳ������ TIMER_A_CMSIS(TIMER_A0_BASE)->R ��TAxR ����д�Ͷ�������
       д-->TIMER_A_CMSIS(TIMER_A0_BASE)->R = 0;
       ��-->temp = TIMER_A_CMSIS(TIMER_A0_BASE)->R;
    ���� SR04 Ӧ���п��� ���üĴ���д���߶����Ի��ʱ����
*/
#include "tim_A0.h"

/*  ������ģʽ  */
void TimA0_Int_Init(uint16_t ccr0, uint16_t psc)
{
    // 1.������ģʽ��ʼ��
    Timer_A_UpModeConfig upConfig;
    upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                      //ʱ��Դ
    upConfig.clockSourceDivider = psc;                                                     //ʱ�ӷ�Ƶ ��Χ1-64
    upConfig.timerPeriod = ccr0;                                                           //�Զ���װ��ֵ��ARR��
    upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                   //���� tim����ж�
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //���� ccr0�����ж�
    upConfig.timerClear = TIMER_A_DO_CLEAR;                                                // Clear value

    // 2.��ʼ����ʱ��A
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

    // 3.ѡ��ģʽ��ʼ��������ʼ����ʱ����Ҫʹ�øú���������
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    // 4.����Ƚ��жϱ�־λ
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // 5.������ʱ���˿��ж�
    Interrupt_enableInterrupt(INT_TA0_0);

		/*  6.�������ж� (����������һ�����ͺ���) */
//		MAP_Interrupt_enableMaster();
}



/*
*  TIMA0 �жϷ�����
*  
*	
*/
void TA0_0_IRQHandler(void)
{
		/* ����жϱ�־λ */
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /* ��ʼ����û����� */
    GetMotorPulse();
    
    /* ��������û����� */
}

void TA0_N_IRQHandler(void)
{
		/* ����жϱ�־λ */
		MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    /* ��ʼ����û����� */
		
    /* ��������û����� */
}


