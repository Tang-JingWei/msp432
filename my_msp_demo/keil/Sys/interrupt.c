#include "sysinit.h"

/*
*
* �����жϷ������ĺ������� startup_msp432p401r_uvision.s �����ļ���
* 
* ��C�ļ�����жϷ�����
*
*/


/*
*  PORT1 �ⲿ�жϷ�����
*  
*	
*/
// void PORT1_IRQHandler(void)
// {
// 	uint16_t status;
// 	//uint16_t i;
	
// 	/*  ����PORT�˿ڣ�GPIO_getEnabledInterruptStatus()�᷵�ش������жϵ�PIN����  */
// 	status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
	
// 	/*  ���ָ��PIN���жϱ�־λ  */
// 	GPIO_clearInterruptFlag(GPIO_PORT_P1,status);

// 	if(status & GPIO_PIN1)
// 	{		
// 		LED_RED_On();
// 	}

// 	if(status & GPIO_PIN4)
// 	{
// 		LED_RED_Off();
// 	}
// }


/*
�������ڽ����ж�
		1��������� UART_receiveData() ��������ȡ���ռĴ����Ļ���
			 �Ͳ�������жϱ�־λ���ò������Զ�ʹ��־λ���
		2�����û�� UART_receiveData() ��������ô��Ҫ�� UART_clearInterruptFlag() �ֶ����

*/









