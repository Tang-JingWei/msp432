#include "sysinit.h"

/*
*
* 具体中断服务函数的函数名在 startup_msp432p401r_uvision.s 启动文件中
* 
* 该C文件存放中断服务函数
*
*/


/*
*  PORT1 外部中断服务函数
*  
*	
*/
// void PORT1_IRQHandler(void)
// {
// 	uint16_t status;
// 	//uint16_t i;
	
// 	/*  传入PORT端口，GPIO_getEnabledInterruptStatus()会返回触发了中断的PIN引脚  */
// 	status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
	
// 	/*  清除指定PIN的中断标志位  */
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
！！串口接收中断
		1）如果是用 UART_receiveData() 函数来读取接收寄存器的话，
			 就不用清除中断标志位，该操作会自动使标志位清除
		2）如果没用 UART_receiveData() 函数，那么就要用 UART_clearInterruptFlag() 手动清除

*/









