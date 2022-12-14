/*********************************************************************************************************/

/**************************************         tim32_0          *******************************************/

/*

笔记：
    1、模式
    ? Free-running mode: The counter wraps after reaching its zero value, and continues to count down from
      the maximum value. This is the default mode.
    ? Periodic timer mode: The counter generates an interrupt at a constant interval, reloading the original
      value after wrapping past zero.
    2、时钟来源
    ? MCLK
    ? MCLK divided by 16, generated by 4 bits of prescale
    ? MCLK divided by 256, generated by a total of 8 bits of prescale
*/

#include "tim32_0.h"

/*  TIM32_0 Periodic mode模式  */
void Tim32_0_Int_Init(uint32_t aar, uint8_t psc)
{
  Timer32_initModule(TIMER32_0_BASE, psc, TIMER32_32BIT, TIMER32_PERIODIC_MODE);

  Timer32_setCount(TIMER32_0_BASE, aar);
  Timer32_enableInterrupt(TIMER32_0_BASE);
  Timer32_startTimer(TIMER32_0_BASE, false); //连续计数模式--false 单次模式--true

  Interrupt_enableInterrupt(INT_T32_INT1);
}

/* Timer32_0 ISR */
void T32_INT1_IRQHandler(void)
{
  /*  清除溢出中断  */
  Timer32_clearInterruptFlag(TIMER32_0_BASE);

  /***********开始填充用户代码************/
  
  /* 可获得当前速度once 和 累计脉冲total */
  // GetMotorPulse();

  /* 陀螺仪 yaw角 */
  // yaw = Mpu_Read_Yaw();
  // roll = Mpu_Read_Roll();
  // pitch = Mpu_Read_Pitch();
  // SendDataToLcd("Sysinfo.yaw.txt=\"%.2f\"", yaw);
  // SendDataToLcd("Sysinfo.roll.txt=\"%.2f\"", roll);
  // SendDataToLcd("Sysinfo.pitch.txt=\"%.2f\"", pitch);

  /* 直走 */
  if (Line_flag == 1)
  {
    /*  使能状态下才控制  */
    if (left_en_flag == 1 || right_en_flag == 1)
    {
      /* 循迹环pwm补偿 加速*/
      Gray_Search();

      /* 循迹补偿 所乘系数使得循迹补偿不会过大，导致低速时过调，平滑输出 */
      Now_Left_Pwm += (Search_Buchang * (float)(Now_Left_Pwm / 1800.0));
      Now_Right_Pwm -= (Search_Buchang * (float)(Now_Left_Pwm / 1800.0));

      /* 经过pid控制器输出控制pwm信号 */
      Speeds_Pid_Control();

      /* 循迹补偿 */
      Now_Left_Pwm += Search_Buchang;
      Now_Right_Pwm -= Search_Buchang;

      Motors_Control(Now_Left_Dir, Now_Left_Pwm, Now_Right_Dir, Now_Right_Pwm);

      /* 定点位置在误差允许范围判断 左右轮同时判断*/
      if (
          ((Left_Total_Pulse <= (LineGo_Target_Maichong + 100)) && (Left_Total_Pulse >= (LineGo_Target_Maichong - 100))) ||
          ((Right_Total_Pulse <= (LineGo_Target_Maichong + 100)) && (Right_Total_Pulse >= (LineGo_Target_Maichong - 100))))
      {
        stop_count++; //摆动次数
        if (stop_count == 10)
        {
          Line_flag = 0;  //直走完成，到目标点了
          Stop_flag = 1;  //小车处于停止状态
          stop_count = 0; //置位计数

          Motor_L_stop(); //小车停止，stop
          Motor_R_stop();
        }
      }
    }
  }

  /* 转弯 */
  if (Turn_flag == 1)
  {
    /*  使能状态下才控制  */
    if (left_en_flag == 1 || right_en_flag == 1)
    {
      /* 读取当前的航向角 */
      now_car_yaw = Mpu_Read_Yaw();

      float delt_yaw; //角度差值
      delt_yaw = now_car_yaw - last_car_yaw;

      /* 陀螺仪控制转弯 */
      if ((delt_yaw <= (target_yaw + 2)) && (delt_yaw >= (target_yaw - 2)))
      {
        /* spin_count控制最长转弯时间 */
        spin_count++;
        if (spin_count == 2)
        {
          Turn_flag = 0;  //转弯完成
          Stop_flag = 1;  //停止位 置1
          spin_count = 0; //计数归0

          Motor_L_stop(); //小车停止，stop
          Motor_R_stop();
        }
      }
    }
  }

  /***********结束填充用户代码************/
}
