/*********************************************************************************************************/

/**************************************         SYSTICK          *****************************************/

/*

�ʼǣ�
  1����Ϊsystick��֧���ⲿʱ�ӣ�����systick��ʱ��Դ��MCLK=48M



*/

#include "bsp_systick.h"

/* systick��ʼ�� ����ֵ����С��0xffffff��24λ������  */
void Systick_Init(uint32_t period)
{
  /*  ģ���ʼ��  */
  SysTick_enableModule();

  /*  ����periodֵ ʱ��Դ��MCLK=48M */
  SysTick_setPeriod(period);

  /*  �����ж�  */
  SysTick_enableInterrupt();
}

/*
 *  SYSTICK �жϷ�����
 *
 *
 */
void SysTick_Handler(void)
{
  /* ����Ҫ����ж������־  */

  /*��ʼ����û�����*/
  /*��������û�����*/
}
