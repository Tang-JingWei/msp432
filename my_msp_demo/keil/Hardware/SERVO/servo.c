#include "servo.h"

float servo_angle = 0;

/**
 * @brief  ���ת�����
 * @param  code �������, angle �Ƕ�(+-)
 * @retval ��
 */
void Servo_Move(servo_code code, float angle)
{
  uint16_t pwm;

  /* �޷� */
  if (angle > 180)
    angle = 180;
  if (angle < 0)
    angle = 0;

  if (code == down_servo) /* �¶�� ���� */
  {
    if (angle < 0)
    {
      pwm = (-1) * angle * 11.11 + 500; /* �ǶȻ����pwm */
    }
    else
    {
      pwm = angle * 11.11 + 500; /* �ǶȻ����pwm */
    }

    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, pwm);
  }

  else if (code == up_servo) /* �϶�� ���� */
  {
    if (angle < 0)
    {
      pwm = (-1) * angle * 11.11 + 500; /* �ǶȻ����pwm */
    }
    else
    {
      pwm = angle * 11.11 + 500; /* �ǶȻ����pwm */
    }

    MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, pwm);
  }
}

/**
 * @brief  ���¶����λ
 * @param  ��
 * @retval ��
 */
void Servos_Relocate()
{
  /* pwm�ź� */
}

/**
 * @brief  ���¶����ʱ����ʼ��
 * @param  ��
 * @retval ��
 */
void Servos_Timer_Init()
{
  /* ���pwm�������� TimerA1
     psc: 48
     arr: 20000
     duty: 0 (��ʼ��ʱΪ0)
  */

  Timer_A_PWMConfig TimA1_servo_PWMConfig;
 
  /*  ��ʼ������  */
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

  /*  ���ýṹ��  */
  TimA1_servo_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;             //ʱ��Դ
  TimA1_servo_PWMConfig.clockSourceDivider = 48;                             //ʱ�ӷ�Ƶ ��Χ1-64
  TimA1_servo_PWMConfig.timerPeriod = 20000;                                 //�Զ���װ��ֵ��ARR��
  TimA1_servo_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;   //���ģʽ
  TimA1_servo_PWMConfig.dutyCycle = 0;                                       //ռ�ձ�

  /*  �¶��  */
  TimA1_servo_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //ͨ��һ �����ţ�
  Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_servo_PWMConfig); 

  /*  �϶��  */
  TimA1_servo_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //ͨ���� �����ţ�
  Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_servo_PWMConfig); 


}
