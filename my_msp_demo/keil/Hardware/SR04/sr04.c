/*
	SR04 
	
	1��ѡ�ö�ʱ�� Timer_32_0������Ƶ�ʿ���Ϊ 1M Hz����֤����һ������Ҫ 1us��
	2���� (float)measure * 17.0 / 1000.0 ��ʽ���ؾ���

*/

#include "sr04.h"

extern int a;
static float distance_temp = 0;
static float distance_last = 0;
float sr04_distance = 0;		//����

void UltrasonicWave_TimerInit(void)
{
	/* sr04 ����ʽ */
	TimA2_Cap_Init();

	//��������������������������������������������������������������ʽ����������������������������������������������������������������������������������
	// // 1.������ģʽ��ʼ��
	// Timer_A_UpModeConfig upConfig;
	// upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;																			 //ʱ��Դ
	// upConfig.clockSourceDivider = 48;																											 //ʱ�ӷ�Ƶ ��Χ1-64
	// upConfig.timerPeriod = 65535;																													 //�Զ���װ��ֵ��ARR��
	// upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;									 //���� tim����ж�
	// upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //���� ccr0�����ж�
	// upConfig.timerClear = TIMER_A_DO_CLEAR;																								 // Clear value

	// // 2.��ʼ����ʱ��A
	// Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

	// // 4.����Ƚ��жϱ�־λ
	// Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	// // 5.������ʱ���˿��ж�
	// Interrupt_enableInterrupt(INT_TA0_0);
	//��������������������������������������������������������������������������������������������������������������������������������������������
}

void UltrasonicWave_StartMeasure(void)
{
	GPIO_setOutputHighOnPin(SR04T_GPIO_Port, SR04T_Pin);
	delay_ms(1);
	GPIO_setOutputLowOnPin(SR04T_GPIO_Port, SR04T_Pin);
}

float UltrasonicWave_Measure(void)
{
	uint16_t time_node1 = 0;
	uint16_t time_node2 = 0;
	uint16_t measure = 0;

	/*  ����GPIOģʽ  */
	MAP_GPIO_setAsOutputPin(SR04T_GPIO_Port, SR04T_Pin);
	// MAP_GPIO_setAsInputPin(SR04R_GPIO_Port, SR04R_Pin);

	/*  trigger ���䳬����  */
	UltrasonicWave_StartMeasure();

	//����������������������������������������������(�ж�)ʽ����������������������������������������������������������������������
	// tim_A2 interrupt


	//����������������������������������������������ʽ������������������������������������������������������������

	// /*  ��ͣ��ʱ��  */
	// Timer_A_stopTimer(TIMER_A0_BASE); 
	// /*  �ȴ� echo�ĸߵ�ƽ����  */
	// while (GPIO_getInputPinValue(SR04R_GPIO_Port, SR04R_Pin) == 0)

	// /*  TIMER_A0����������  */
	// TIMER_A_CMSIS(TIMER_A0_BASE)->R = 0;
	// /*  ѡ��upģʽ��ʼ��������ʼ����ʱ����Ҫʹ�øú��������� */
	// Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
	// /*  ��¼��ʼλ��  */
	// time_node1 = Timer_A_getCounterValue(TIMER_A0_BASE);
	// /*  �ȴ� echo�ĸߵ�ƽ����  */
	// while (GPIO_getInputPinValue(SR04R_GPIO_Port, SR04R_Pin) == 1)  
	// /*  ��¼����λ��  */
	// time_node2 = Timer_A_getCounterValue(TIMER_A0_BASE);

	// /*  �����ֵ  */
	// measure = time_node2 - time_node1;
	// return (float)measure * 17.0 / 1000.0; //�˴���λת��Ϊcm

}
