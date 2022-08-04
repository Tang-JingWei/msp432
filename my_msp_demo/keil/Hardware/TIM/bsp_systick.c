/*********************************************************************************************************/

/**************************************         SYSTICK          *****************************************/

/*

笔记：
  1、因为systick不支持外部时钟，所以systick的时钟源是MCLK=48M



*/

#include "bsp_systick.h"

/* systick初始化 计数值必须小于0xffffff（24位计数）  */
void Systick_Init(uint32_t period)
{
  /*  模块初始化  */
  SysTick_enableModule();

  /*  设置period值 时钟源是MCLK=48M */
  SysTick_setPeriod(period);

  /*  开启中断  */
  SysTick_enableInterrupt();
}

/*
 *  SYSTICK 中断服务函数
 *
 *
 */
void SysTick_Handler(void)
{
  /* 不需要清除中断溢出标志  */

  /*开始填充用户代码*/
  /*结束填充用户代码*/
}
