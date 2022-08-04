#include "servo.h"

float servo_angle = 0;

/**
 * @brief  舵机转向控制
 * @param  code 舵机代号, angle 角度(+-)
 * @retval 无
 */
void Servo_Move(servo_code code, float angle)
{
  uint16_t pwm;

  /* 限幅 */
  if (angle > 180)
    angle = 180;
  if (angle < 0)
    angle = 0;

  if (code == down_servo) /* 下舵机 左右 */
  {
    if (angle < 0)
    {
      pwm = (-1) * angle * 11.11 + 500; /* 角度换算成pwm */
    }
    else
    {
      pwm = angle * 11.11 + 500; /* 角度换算成pwm */
    }

    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, pwm);
  }

  else if (code == up_servo) /* 上舵机 上下 */
  {
    if (angle < 0)
    {
      pwm = (-1) * angle * 11.11 + 500; /* 角度换算成pwm */
    }
    else
    {
      pwm = angle * 11.11 + 500; /* 角度换算成pwm */
    }

    MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, pwm);
  }
}

/**
 * @brief  上下舵机归位
 * @param  无
 * @retval 无
 */
void Servos_Relocate()
{
  /* pwm信号 */
}

/**
 * @brief  上下舵机定时器初始化
 * @param  无
 * @retval 无
 */
void Servos_Timer_Init()
{
  /* 舵机pwm输入利用 TimerA1
     psc: 48
     arr: 20000
     duty: 0 (初始化时为0)
  */

  Timer_A_PWMConfig TimA1_servo_PWMConfig;
 
  /*  初始化引脚  */
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

  /*  配置结构体  */
  TimA1_servo_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;             //时钟源
  TimA1_servo_PWMConfig.clockSourceDivider = 48;                             //时钟分频 范围1-64
  TimA1_servo_PWMConfig.timerPeriod = 20000;                                 //自动重装载值（ARR）
  TimA1_servo_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;   //输出模式
  TimA1_servo_PWMConfig.dutyCycle = 0;                                       //占空比

  /*  下舵机  */
  TimA1_servo_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //通道一 （引脚）
  Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_servo_PWMConfig); 

  /*  上舵机  */
  TimA1_servo_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //通道二 （引脚）
  Timer_A_generatePWM(TIMER_A1_BASE, &TimA1_servo_PWMConfig); 


}
