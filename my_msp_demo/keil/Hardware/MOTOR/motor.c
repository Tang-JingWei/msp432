#include "motor.h"

/* 电机的编码器会记录或者运算到负数，不能用uint 只能用int */
int8_t Motor_Direction = 0;      //电机转动方向
int32_t Motor_Capture_Count = 0; //当前计数器值
float Motor_Shaft_Speed = 0;     //转轴速度

long Left_Once_Pulse = 0; //左右轮单次读取编码器的脉冲
long Right_Once_Pulse = 0;
long Left_Total_Pulse = 0; //左右轮累计获得脉冲数
long Right_Total_Pulse = 0;

/*  记录 电机 方向(DIR)|速度(PWM) --全局调用-- */
uint8_t Now_Left_Dir = 0;
uint8_t Now_Right_Dir = 0;
uint16_t Now_Left_Pwm = 0;
uint16_t Now_Right_Pwm = 0;

uint8_t Line_flag = 0, Turn_flag = 0, Stop_flag = 0; //方向flag
uint8_t spin_start_flag, spin_succeed_flag;          //转弯flag
uint8_t left_en_flag = 0, right_en_flag = 0;         //左右电机使能flag

double Left_journey_pulse, Right_journey_pulse; //左右轮行走距离

/* 直行目标脉冲数 左右轮相同 */
double LineGo_Target_Maichong;

/* 急转弯目标脉冲数 左右轮一个负一个正*/
double SpinLeft90_Target_Maichong = 2000;  //待弃用
double SpinRight90_Target_Maichong = 1700; //待弃用
double SpinBack180_Target_Maichong = 2800; //待弃用

float last_car_yaw = 0; //记录转弯前时刻的角度
float now_car_yaw = 0;  //转弯后的当前角度
float target_yaw = 0;   //转弯目标角度

uint16_t stop_count, spin_count; // pid调整末尾时允许的摆动次数

/*  电机 PID 相关  */
float L_Pwm_Outval, L_Location_Outval; //经过pid后的控制量
float R_Pwm_Outval, R_Location2_Outval;

/*********************************************/

/*************** tb6612驱动	控制 *******************/

/**
 * @brief  电机相关IO口初始化
 * @param  无
 * @retval 无
 */
void Motors_IO_Init(void)
{
  /* AIN1 AIN2 */
  GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5 | GPIO_PIN6);

  /* BIN1 BIN2 */
  GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
  GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN1);
}

/**
 * @brief  电机pwm输入控制 定时器初始化
 * @param  无
 * @retval 无
 */
void Motors_Timer_Init(void)
{
  /* 电机pwm输入利用 TimerA0
     psc: 48
     arr: 2000
     duty: 0 (初始化时为0)
  */

  Timer_A_PWMConfig TimA0_PWMConfig;

  /*  初始化引脚 分别是PWM_A  PWM_B */
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

  /*  定时器PWM初始化  */
  TimA0_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;           //时钟源
  TimA0_PWMConfig.clockSourceDivider = 48;                           //时钟分频 范围1-64
  TimA0_PWMConfig.timerPeriod = 2000 - 1;                            //自动重装载值（ARR）
  TimA0_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET; //输出模式
  TimA0_PWMConfig.dutyCycle = 0;                                     //占空比

  TimA0_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //通道一 （引脚选择）
  Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);

  TimA0_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //通道一 （引脚选择）
  Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);
}

/**
  * @brief  电机总控
  * @param  左方向  左PWM, 右方向, 右PWM
      方向:1、STP（0）
         2、FWD（1）
         3、BCK（2）
  * @retval 无
  */
void Motors_Control(motor_dir L_dir, uint16_t L_pwm, motor_dir R_dir, uint16_t R_pwm)
{
  /* 定时器 period值 设置为 1000 */

  // pwm 输出 PWM输出一定要先打开
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

  /*  左轮  */
  Set_Left_Dir(L_dir);
  if (L_pwm >= PWM_PERIOD_COUNT - 100) //定上限
  {
    L_pwm = PWM_PERIOD_COUNT - 100;
  }
  Set_Left_Speed(L_pwm);

  /*  右轮  */
  Set_Right_Dir(R_dir);
  if (R_pwm >= PWM_PERIOD_COUNT - 100) //定上限
  {
    R_pwm = PWM_PERIOD_COUNT - 100;
  }
  Set_Right_Speed(R_pwm);
}

/**
 * @brief  直走函数
 * @param  期望直行的距离 cm 可为负数，即倒退
 * @retval 无
 */
void Car_Go(int32_t desire_cm)
{
  int32_t Target_Maichong = 0;

  Line_flag = 1;
  Stop_flag = 0;

  spin_start_flag = 0;
  spin_succeed_flag = 0;

  /* 将目标距离转换成脉冲数 */
  LineGo_Target_Maichong = (desire_cm / (4.4 * 3.14)) * (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION);
  // printf("maichong: %d\r\n",Target_Maichong);

  /*  PID 设置target */
  set_pid_target(&pid_left_location, LineGo_Target_Maichong);
  set_pid_target(&pid_right_location, LineGo_Target_Maichong);

  /* msp432暂时不知道如何暂停pwm输出，所以初始化的时候已经将pwm打开，后续不会涉及pwm开和关，只有改变大小 */
  // Left_Enable(); //使能输出，准备前进
  // Right_Enable();
  left_en_flag = 1;
  right_en_flag = 1;

  Left_Total_Pulse = 0;
  Right_Total_Pulse = 0;
}

/**
 * @brief  急转弯
 * @param  转弯角度 left_90 right_90 back_180
 * @retval 无
 */
void Spin_Turn(spin_angle angle)
{
  Line_flag = 0; //直行标志 置0
  Stop_flag = 0;

  Turn_flag = 1; //转弯位 置1
  // spin_succeed_flag = 0; //开始转弯，成功标志位 置0

  /*
    note：车身左转角度增大 右转角度减小 看情况调整
  */

  /* 左 90 */
  if (angle == left_90)
  {
    last_car_yaw = Mpu_Read_Yaw();
    target_yaw = 12;
    Motors_Control(MOTOR_BCK, 700, MOTOR_FWD, 700);
  }
  /* 右 90 */
  else if (angle == right_90)
  {
    last_car_yaw = Mpu_Read_Yaw();
    target_yaw = -12;
    Motors_Control(MOTOR_FWD, 700, MOTOR_BCK, 700);
  }
  /* 掉头 180 */
  else if (angle == back_180)
  {
    last_car_yaw = Mpu_Read_Yaw();
    target_yaw = 23;
    Motors_Control(MOTOR_BCK, 700, MOTOR_FWD, 700); //掉头采用左转
  }
}

/**
 * @brief  匀速直走
 * @param  speed: 速度（单位时间脉冲数）
 * @retval 无
 */
void YunSu_GO(float speed)
{
  Line_flag = 1;
  Stop_flag = 0;

  set_pid_target(&pid_left_speed, speed);
  set_pid_target(&pid_right_speed, speed);

  left_en_flag = 1; //使能输出，准备前进
  right_en_flag = 1;

  Left_Total_Pulse = 0;
  Right_Total_Pulse = 0;
}

/**
 * @brief  设置电机速度
 * @param  x_pwm: 速度（占空比）
 * @retval 无
 */
void Set_Left_Speed(uint16_t L_pwm)
{
  /* 改变compare值以改变占空比 */
  MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, L_pwm);
  Now_Left_Pwm = L_pwm;
}

void Set_Right_Speed(uint16_t R_pwm)
{
  /* 改变compare值以改变占空比 */
  MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, R_pwm);
  Now_Right_Pwm = R_pwm;
}

/**
 * @brief  设置电机方向
 * @param  x_dir
 * @retval 无
 */
void Set_Left_Dir(motor_dir L_dir)
{
  if (L_dir == MOTOR_FWD) // 1
  {
    Motor_L_forward();
    Now_Left_Dir = MOTOR_FWD;
  }
  else if (L_dir == MOTOR_BCK) // 2
  {
    Motor_L_back();
    Now_Left_Dir = MOTOR_BCK;
  }
  else // 0
  {
    Motor_L_stop();
    Now_Left_Dir = MOTOR_STP;
  }
}

void Set_Right_Dir(motor_dir R_dir)
{
  if (R_dir == MOTOR_FWD) // 1
  {
    Motor_R_forward();
    Now_Right_Dir = MOTOR_FWD;
  }
  else if (R_dir == MOTOR_BCK) // 2
  {
    Motor_R_back();
    Now_Right_Dir = MOTOR_BCK;
  }
  else // 0
  {
    Motor_R_stop();
    Now_Right_Dir = MOTOR_STP;
  }
}

/**
 * @brief  基础电机驱动
 * @param  无
 * @retval 无
 */
void Motor_L_forward()
{
  /*  AIN1---AIN2  */
  /*   0------1  */
  GPIO_setOutputLowOnPin(AIN1_GPIO_Port, AIN1_Pin);
  GPIO_setOutputHighOnPin(AIN2_GPIO_Port, AIN2_Pin);
  left_en_flag = 1;
}

void Motor_L_back()
{
  /*  AIN1---AIN2  */
  /*   1------0  */
  GPIO_setOutputHighOnPin(AIN1_GPIO_Port, AIN1_Pin);
  GPIO_setOutputLowOnPin(AIN2_GPIO_Port, AIN2_Pin);
  left_en_flag = 1;
}

void Motor_L_stop()
{
  /*  AIN1---AIN2  */
  /*   0------0  */
  GPIO_setOutputLowOnPin(AIN1_GPIO_Port, AIN1_Pin);
  GPIO_setOutputLowOnPin(AIN2_GPIO_Port, AIN2_Pin);
  left_en_flag = 0;
}

void Motor_R_forward()
{
  /*  BIN1---BIN2  */
  /*   0------1  */
  GPIO_setOutputLowOnPin(BIN1_GPIO_Port, BIN1_Pin);
  GPIO_setOutputHighOnPin(BIN2_GPIO_Port, BIN2_Pin);
  right_en_flag = 1;
}

void Motor_R_back()
{
  /*  BIN1---BIN2  */
  /*   1------0  */
  GPIO_setOutputHighOnPin(BIN1_GPIO_Port, BIN1_Pin);
  GPIO_setOutputLowOnPin(BIN2_GPIO_Port, BIN2_Pin);
  right_en_flag = 1;
}

void Motor_R_stop()
{
  /*  BIN1---BIN2  */
  /*   0------0  */
  GPIO_setOutputLowOnPin(BIN1_GPIO_Port, BIN1_Pin);
  GPIO_setOutputLowOnPin(BIN2_GPIO_Port, BIN2_Pin);
  // right_en_flag = 0;
}

/************************** 左右电机 距离控制速度pid控制 *****************************/
/**
 * @brief  pid控制 路程环+速度环
 * @param  无
 * @retval 无
 */
void Speeds_Pid_Control()
{
  float temp_pwm = 0, temp_speed = 0, now_left_speed = 0, now_right_speed = 0;

  /* 左电机速度pid */
  Left_journey_pulse = Left_Total_Pulse;
  temp_speed = Location_pid_realize(&pid_left_location, Left_journey_pulse);
  set_pid_target(&pid_left_speed, temp_speed);
  now_left_speed = (float)(Left_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));
  temp_pwm = speed_pid_realize(&pid_left_speed, now_left_speed);
  if (temp_pwm > 0)
  {
    Now_Left_Dir = MOTOR_FWD;
    Now_Left_Pwm = temp_pwm;
  }
  else if (temp_pwm < 0)
  {
    Now_Left_Dir = MOTOR_BCK;
    Now_Left_Pwm = -temp_pwm;
  }
  else
  {
    Now_Left_Dir = MOTOR_STP;
    Now_Left_Pwm = 0;
  }

  /* 右电机速度pid */
  Right_journey_pulse = Right_Total_Pulse;
  temp_speed = Location_pid_realize(&pid_right_location, Right_journey_pulse);
  set_pid_target(&pid_right_speed, temp_speed);
  now_right_speed = (float)(Right_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));
  temp_pwm = speed_pid_realize(&pid_right_speed, now_right_speed);
  if (temp_pwm > 0)
  {
    Now_Right_Dir = MOTOR_FWD;
    Now_Right_Pwm = temp_pwm;
  }
  else if (temp_pwm < 0)
  {
    Now_Right_Dir = MOTOR_BCK;
    Now_Right_Pwm = -temp_pwm;
  }
  else
  {
    Now_Right_Dir = MOTOR_STP;
    Now_Right_Pwm = 0;
  }
  /* 打印测试数据 */
  printf("%lf,%lf,%lf,%lf,%lf\r\n", (100 / (4.4 * 3.14)) * (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION), Left_journey_pulse, Right_journey_pulse, now_left_speed, now_right_speed);
  // printf("%lf,%lf,%lf,%lf,%lf\r\n", temp_speed, now_left_speed, now_right_speed);
}

/************************** 左右电机 角度控制速度pid控制 *****************************/
/**
 * @brief  pid控制
 * @param  无
 * @retval 无
 */
void Angel_Pid_Control()
{
}

/************************** 左右电机 匀速pid控制 *****************************/
/**
 * @brief  pid控制 只有速度环
 * @param  无
 * @retval 无
 */
void YunSu_Pid_Control()
{
  float temp_pwm = 0, now_left_speed = 0, now_right_speed = 0;

  /* 左电机pid */
  now_left_speed = (float)(Left_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));
  temp_pwm = speed_pid_realize(&pid_left_speed, now_left_speed);
  if (temp_pwm > 0)
  {
    Now_Left_Dir = MOTOR_FWD;
    Now_Left_Pwm = temp_pwm;
  }
  else if (temp_pwm < 0)
  {
    Now_Left_Dir = MOTOR_BCK;
    Now_Left_Pwm = -temp_pwm;
  }
  else
  {
    Now_Left_Dir = MOTOR_STP;
    Now_Left_Pwm = 0;
  }

  /* 右电机pid */
  now_right_speed = (float)(Right_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));
  temp_pwm = speed_pid_realize(&pid_right_speed, now_right_speed);
  if (temp_pwm > 0)
  {
    Now_Right_Dir = MOTOR_FWD;
    Now_Right_Pwm = temp_pwm;
  }
  else if (temp_pwm < 0)
  {
    Now_Right_Dir = MOTOR_BCK;
    Now_Right_Pwm = -temp_pwm;
  }
  else
  {
    Now_Right_Dir = MOTOR_STP;
    Now_Right_Pwm = 0;
  }
  /* 打印测试数据 */
  // printf("%lf,%lf,%lf,%lf,%lf\r\n", (100 / (4.4 * 3.14)) * (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION), Left_journey_pulse, Right_journey_pulse, now_left_speed, now_right_speed);
  // printf("%lf,%lf,%lf\r\n", pid_right_speed.target_val, now_left_speed, now_right_speed);
}

/*								编码器									*/
/*******************电机编码器初始化************************/
void Encoder_Configuration(void)
{
  /*配置为浮空输入引脚*/
  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);
  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN7);
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);

  /*开启外部中断*/
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN7); // left motor  P1.6 | P1.7
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7); // right motor P2.6 | P2.7

  /*开启端口中断*/
  Interrupt_enableInterrupt(INT_PORT1);
  Interrupt_enableInterrupt(INT_PORT2);

  /*配置触发方式*/
  GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
  GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN7, GPIO_LOW_TO_HIGH_TRANSITION);
  GPIO_interruptEdgeSelect(GPIO_PORT_P2, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
  GPIO_interruptEdgeSelect(GPIO_PORT_P2, GPIO_PIN7, GPIO_LOW_TO_HIGH_TRANSITION);

  Interrupt_setPriority(INT_TA0_0, 1 << 5);
  Interrupt_setPriority(INT_PORT1, 1 << 5);
  Interrupt_setPriority(INT_PORT2, 1 << 5);

  GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);
  GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN7);
  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN7);
}

/*****************************************************************
 *Function: PORT1_IRQHandler(void)
 *Description:外部中断服务函数  记录左轮速度
 *Input:无
 *Output:无
 *Return:无
 *Others:中断服务函数，在中断内处理程序
 *****************************************************************/
void PORT1_IRQHandler(void)
{
  uint16_t flag;

  /*获取中断状态*/
  flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
  /*清除中断标志位*/
  GPIO_clearInterruptFlag(GPIO_PORT_P1, flag);
  /*左轮编码器线A*/
  if (flag & GPIO_PIN6)
  {
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == 0)
      Left_Once_Pulse++;
    else if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == 1)
      Left_Once_Pulse--;
  }

  /*左轮编码器线B*/
  if (flag & GPIO_PIN7)
  {
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == 0)
      Left_Once_Pulse--;
    else if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6) == 1)
      Left_Once_Pulse++;
  }
}

/*****************************************************************
 *Function: PORT2_IRQHandler(void)
 *Description:外部中断服务函数  记录右轮速度
 *Input:无
 *Output:无
 *Return:无
 *Others:中断服务函数，在中断内处理程序
 *****************************************************************/
void PORT2_IRQHandler(void)
{
  uint16_t flag;

  /*获取中断状态*/
  flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
  /*清除中断标志位*/
  GPIO_clearInterruptFlag(GPIO_PORT_P2, flag);

  /*右轮编码器线A*/
  if (flag & GPIO_PIN7)
  {
    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == 0)
      Right_Once_Pulse++;
    else if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1)
      Right_Once_Pulse--;
  }

  /*右轮编码器线B*/
  if (flag & GPIO_PIN6)
  {
    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == 0)
      Right_Once_Pulse--;
    else if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1)
      Right_Once_Pulse++;
  }
}

/*******************实际运行时读取编码器数值************************/
void GetMotorPulse(void) //读取电机脉冲
{
  Left_Total_Pulse += Left_Once_Pulse; //累计脉冲	完成一次pid之后要清0
  Right_Total_Pulse += Right_Once_Pulse;

  /* 转轴转速 = 单位时间内的计数值 / 编码器总分辨率=(编码器物理分辨率/4) * 时间系数  */
  Motor_Shaft_Speed = (float)(Left_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));
 
  SendDataToLcd("Sysinfo.left_v.txt=\"%.2f\"", Motor_Shaft_Speed);
  // printf("电机转轴处转速：%.2f 转/分 \r\n", Motor_Shaft_Speed);
  // printf("电机输出轴转速：%.2f 转/分 \r\n\n", Motor_Shaft_Speed / 20.0); /* 输出轴转速 = 转轴转速 / 减速比 */

  Motor_Shaft_Speed = (float)(Right_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));

  SendDataToLcd("Sysinfo.right_v.txt=\"%.2f\"", Motor_Shaft_Speed);
  // printf("电机转轴处转速：%.2f 转/分 \r\n", Motor_Shaft_Speed);
  // printf("电机输出轴转速：%.2f 转/分 \r\n\n", Motor_Shaft_Speed / 20.0); /* 输出轴转速 = 转轴转速 / 减速比 */

  // printf("--------------------------------\r\n");

  Left_Once_Pulse = 0;
  Right_Once_Pulse = 0;
}
