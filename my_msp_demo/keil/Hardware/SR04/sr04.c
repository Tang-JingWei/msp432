/*
	SR04 
	
	1、选用定时器 Timer_32_0，计数频率控制为 1M Hz，保证计数一个点需要 1us。
	2、靠 (float)measure * 17.0 / 1000.0 公式返回距离

*/

#include "sr04.h"

extern int a;
static float distance_temp = 0;
static float distance_last = 0;
float sr04_distance = 0;		//距离

void UltrasonicWave_TimerInit(void)
{
	/* sr04 捕获式 */
	TimA2_Cap_Init();

	//—————————————————————————————阻塞式—————————————————————————————————————————
	// // 1.增计数模式初始化
	// Timer_A_UpModeConfig upConfig;
	// upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;																			 //时钟源
	// upConfig.clockSourceDivider = 48;																											 //时钟分频 范围1-64
	// upConfig.timerPeriod = 65535;																													 //自动重装载值（ARR）
	// upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;									 //禁用 tim溢出中断
	// upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //启用 ccr0更新中断
	// upConfig.timerClear = TIMER_A_DO_CLEAR;																								 // Clear value

	// // 2.初始化定时器A
	// Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

	// // 4.清除比较中断标志位
	// Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	// // 5.开启定时器端口中断
	// Interrupt_enableInterrupt(INT_TA0_0);
	//——————————————————————————————————————————————————————————————————————
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

	/*  设置GPIO模式  */
	MAP_GPIO_setAsOutputPin(SR04T_GPIO_Port, SR04T_Pin);
	// MAP_GPIO_setAsInputPin(SR04R_GPIO_Port, SR04R_Pin);

	/*  trigger 发射超声波  */
	UltrasonicWave_StartMeasure();

	//—————————————————————捕获(中断)式测量—————————————————————————————————
	// tim_A2 interrupt


	//—————————————————————阻塞式测量————————————————————————————

	// /*  暂停定时器  */
	// Timer_A_stopTimer(TIMER_A0_BASE); 
	// /*  等待 echo的高电平到来  */
	// while (GPIO_getInputPinValue(SR04R_GPIO_Port, SR04R_Pin) == 0)

	// /*  TIMER_A0计数器清零  */
	// TIMER_A_CMSIS(TIMER_A0_BASE)->R = 0;
	// /*  选择up模式开始计数（初始化定时器后还要使用该函数启动） */
	// Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
	// /*  记录开始位置  */
	// time_node1 = Timer_A_getCounterValue(TIMER_A0_BASE);
	// /*  等待 echo的高电平结束  */
	// while (GPIO_getInputPinValue(SR04R_GPIO_Port, SR04R_Pin) == 1)  
	// /*  记录结束位置  */
	// time_node2 = Timer_A_getCounterValue(TIMER_A0_BASE);

	// /*  计算差值  */
	// measure = time_node2 - time_node1;
	// return (float)measure * 17.0 / 1000.0; //此处单位转换为cm

}
