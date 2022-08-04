#include "key.h"

/*
*  按键初始化函数
*  
*	mode:   0,不开启中断；   1,开启中断
*	
*	！！！注意：
*	端口配置成输入模式，然后再使能中断
*	GPIO_setAsInputPinWithPullUpResistor --> GPIO_enableInterrupt 
*	中断使能只是使能，PIN口仍然是输入模式，所以仍然可以读引脚高低电平。只是增加了一个中断的功能。
*
*/
void KEY_Init(bool mode) //IO初始化
{
	/*  配置上拉输入模式，外部中断方式引脚也要设置成该模式  */
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
	
	/*  模式选择，是否开启外部中断 */
	if(mode == 1)
	{
		//1.配置GPIO位输入模式（上面）
		
		//2.清除中断标志位
		GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN1);
		GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN4);
		
		//3.配置触发方式
		GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);
		GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN4,GPIO_HIGH_TO_LOW_TRANSITION);
		
		//4.配置成外部中断模式（严格来说不能说是配置成外部中断模式，应该说是使能中断功能）
		GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN1);
		GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN4);
		
		//5.开启端口中断
		Interrupt_enableInterrupt(INT_PORT1);
		
		//6.开启总中断
		// Interrupt_enableMaster();
		
		//7.中断服务函数
	}
}

//按键处理函数
//返回按键值
//mode: 0,不支持连续按;1,支持连续按。
//0，没有任何按键按下
//1，KEY1按下
//2，KEY2按下
//注意此函数有响应优先级,KEY1>KEY2!!
uint8_t KEY_Scan(uint8_t mode)
{
	uint16_t i;
	static uint8_t key_up = 1; //按键按松开标志
	if (mode)
		key_up = 1; //支持连按
	if(key_up && (KEY2 == 0 || KEY1 == 0))
	{
		for (i = 0; i < 500; i++); //去抖动
		key_up = 0;
		if(KEY1 == 0)
		{
			LED_RED_On();
			return KEY1_PRES;
		}
		else if(KEY2 == 0)
		{
			LED_RED_Off();
			return KEY2_PRES;
		}
	}
	else if(KEY2 == 1 && KEY1 == 1)
	{
		key_up = 1;
	}
		
	return 0; // 无按键按下
}
