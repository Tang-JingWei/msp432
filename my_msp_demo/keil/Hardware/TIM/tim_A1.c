/*********************************************************************************************************/

/**************************************         TIMA1          *******************************************/

#include "tim_A1.h"


#define CAP_TIMA_SELECTION 			 TIMER_A1_BASE                         //����Ķ�ʱ��
#define CAP_REGISTER_SELECTION   TIMER_A_CAPTURECOMPARE_REGISTER_1     //����Ķ�ʱ��ͨ��
#define CAP_CCR_NUM 					   1                                     //
#define CAP_PORT_PIN             GPIO_PORT_P5, GPIO_PIN6               //����ĸ�������


/*
*
��غ����� 
				MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, i);


*/


/*   timA1 PWM    */
void TimA1_PWM_Init(uint16_t psc, uint16_t ccr0, uint16_t pulse)
{
    /*  ��ʼ������  */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

		/*  ��ʱ��PWM��ʼ��  */
    Timer_A_PWMConfig TimA1_PWMConfig;
    TimA1_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;             //ʱ��Դ
    TimA1_PWMConfig.clockSourceDivider = psc;                            //ʱ�ӷ�Ƶ ��Χ1-64
    TimA1_PWMConfig.timerPeriod = ccr0;                                  //�Զ���װ��ֵ��ARR��
    TimA1_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //ͨ��һ ������ѡ��
    TimA1_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;   //���ģʽ
    TimA1_PWMConfig.dutyCycle = pulse;                                   //ռ�ձ�

		/*  ��ʼ���ȽϼĴ����Բ��� PWM1��
    API��˵��ʹ����������Ϳ����˶�ʱ����Generate a PWM with timer running in up mode�� */
    Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_PWMConfig); 
}
/*********************************************************************************************************/


/*
*  TIMA1 �жϷ�����
*  
*	
*/
void TA1_0_IRQHandler(void)
{
		/* ����жϱ�־λ */
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /* ��ʼ����û����� */
		

    /* ��������û����� */
}

void TA1_N_IRQHandler(void)
{
		/* ����жϱ�־λ */
		MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    /* ��ʼ����û����� */
		
    
    /* ��������û����� */
}


