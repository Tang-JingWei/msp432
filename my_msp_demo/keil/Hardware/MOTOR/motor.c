#include "motor.h"

/* ����ı��������¼�������㵽������������uint ֻ����int */
int8_t Motor_Direction = 0;      //���ת������
int32_t Motor_Capture_Count = 0; //��ǰ������ֵ
float Motor_Shaft_Speed = 0;     //ת���ٶ�

long Left_Once_Pulse = 0; //�����ֵ��ζ�ȡ������������
long Right_Once_Pulse = 0;
long Left_Total_Pulse = 0; //�������ۼƻ��������
long Right_Total_Pulse = 0;

/*  ��¼ ��� ����(DIR)|�ٶ�(PWM) --ȫ�ֵ���-- */
uint8_t Now_Left_Dir = 0;
uint8_t Now_Right_Dir = 0;
uint16_t Now_Left_Pwm = 0;
uint16_t Now_Right_Pwm = 0;

uint8_t Line_flag = 0, Turn_flag = 0, Stop_flag = 0; //����flag
uint8_t spin_start_flag, spin_succeed_flag;          //ת��flag
uint8_t left_en_flag = 0, right_en_flag = 0;         //���ҵ��ʹ��flag

double Left_journey_pulse, Right_journey_pulse; //���������߾���

/* ֱ��Ŀ�������� ��������ͬ */
double LineGo_Target_Maichong;

/* ��ת��Ŀ�������� ������һ����һ����*/
double SpinLeft90_Target_Maichong = 2000;  //������
double SpinRight90_Target_Maichong = 1700; //������
double SpinBack180_Target_Maichong = 2800; //������

float last_car_yaw = 0; //��¼ת��ǰʱ�̵ĽǶ�
float now_car_yaw = 0;  //ת���ĵ�ǰ�Ƕ�
float target_yaw = 0;   //ת��Ŀ��Ƕ�

uint16_t stop_count, spin_count; // pid����ĩβʱ����İڶ�����

/*  ��� PID ���  */
float L_Pwm_Outval, L_Location_Outval; //����pid��Ŀ�����
float R_Pwm_Outval, R_Location2_Outval;

/*********************************************/

/*************** tb6612����	���� *******************/

/**
 * @brief  ������IO�ڳ�ʼ��
 * @param  ��
 * @retval ��
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
 * @brief  ���pwm������� ��ʱ����ʼ��
 * @param  ��
 * @retval ��
 */
void Motors_Timer_Init(void)
{
  /* ���pwm�������� TimerA0
     psc: 48
     arr: 2000
     duty: 0 (��ʼ��ʱΪ0)
  */

  Timer_A_PWMConfig TimA0_PWMConfig;

  /*  ��ʼ������ �ֱ���PWM_A  PWM_B */
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

  /*  ��ʱ��PWM��ʼ��  */
  TimA0_PWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;           //ʱ��Դ
  TimA0_PWMConfig.clockSourceDivider = 48;                           //ʱ�ӷ�Ƶ ��Χ1-64
  TimA0_PWMConfig.timerPeriod = 2000 - 1;                            //�Զ���װ��ֵ��ARR��
  TimA0_PWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET; //���ģʽ
  TimA0_PWMConfig.dutyCycle = 0;                                     //ռ�ձ�

  TimA0_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2; //ͨ��һ ������ѡ��
  Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);

  TimA0_PWMConfig.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1; //ͨ��һ ������ѡ��
  Timer_A_generatePWM(TIMER_A0_BASE, &TimA0_PWMConfig);
}

/**
  * @brief  ����ܿ�
  * @param  ����  ��PWM, �ҷ���, ��PWM
      ����:1��STP��0��
         2��FWD��1��
         3��BCK��2��
  * @retval ��
  */
void Motors_Control(motor_dir L_dir, uint16_t L_pwm, motor_dir R_dir, uint16_t R_pwm)
{
  /* ��ʱ�� periodֵ ����Ϊ 1000 */

  // pwm ��� PWM���һ��Ҫ�ȴ�
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

  /*  ����  */
  Set_Left_Dir(L_dir);
  if (L_pwm >= PWM_PERIOD_COUNT - 100) //������
  {
    L_pwm = PWM_PERIOD_COUNT - 100;
  }
  Set_Left_Speed(L_pwm);

  /*  ����  */
  Set_Right_Dir(R_dir);
  if (R_pwm >= PWM_PERIOD_COUNT - 100) //������
  {
    R_pwm = PWM_PERIOD_COUNT - 100;
  }
  Set_Right_Speed(R_pwm);
}

/**
 * @brief  ֱ�ߺ���
 * @param  ����ֱ�еľ��� cm ��Ϊ������������
 * @retval ��
 */
void Car_Go(int32_t desire_cm)
{
  int32_t Target_Maichong = 0;

  Line_flag = 1;
  Stop_flag = 0;

  spin_start_flag = 0;
  spin_succeed_flag = 0;

  /* ��Ŀ�����ת���������� */
  LineGo_Target_Maichong = (desire_cm / (4.4 * 3.14)) * (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION);
  // printf("maichong: %d\r\n",Target_Maichong);

  /*  PID ����target */
  set_pid_target(&pid_left_location, LineGo_Target_Maichong);
  set_pid_target(&pid_right_location, LineGo_Target_Maichong);

  /* msp432��ʱ��֪�������ͣpwm��������Գ�ʼ����ʱ���Ѿ���pwm�򿪣����������漰pwm���͹أ�ֻ�иı��С */
  // Left_Enable(); //ʹ�������׼��ǰ��
  // Right_Enable();
  left_en_flag = 1;
  right_en_flag = 1;

  Left_Total_Pulse = 0;
  Right_Total_Pulse = 0;
}

/**
 * @brief  ��ת��
 * @param  ת��Ƕ� left_90 right_90 back_180
 * @retval ��
 */
void Spin_Turn(spin_angle angle)
{
  Line_flag = 0; //ֱ�б�־ ��0
  Stop_flag = 0;

  Turn_flag = 1; //ת��λ ��1
  // spin_succeed_flag = 0; //��ʼת�䣬�ɹ���־λ ��0

  /*
    note��������ת�Ƕ����� ��ת�Ƕȼ�С ���������
  */

  /* �� 90 */
  if (angle == left_90)
  {
    last_car_yaw = Mpu_Read_Yaw();
    target_yaw = 12;
    Motors_Control(MOTOR_BCK, 700, MOTOR_FWD, 700);
  }
  /* �� 90 */
  else if (angle == right_90)
  {
    last_car_yaw = Mpu_Read_Yaw();
    target_yaw = -12;
    Motors_Control(MOTOR_FWD, 700, MOTOR_BCK, 700);
  }
  /* ��ͷ 180 */
  else if (angle == back_180)
  {
    last_car_yaw = Mpu_Read_Yaw();
    target_yaw = 23;
    Motors_Control(MOTOR_BCK, 700, MOTOR_FWD, 700); //��ͷ������ת
  }
}

/**
 * @brief  ����ֱ��
 * @param  speed: �ٶȣ���λʱ����������
 * @retval ��
 */
void YunSu_GO(float speed)
{
  Line_flag = 1;
  Stop_flag = 0;

  set_pid_target(&pid_left_speed, speed);
  set_pid_target(&pid_right_speed, speed);

  left_en_flag = 1; //ʹ�������׼��ǰ��
  right_en_flag = 1;

  Left_Total_Pulse = 0;
  Right_Total_Pulse = 0;
}

/**
 * @brief  ���õ���ٶ�
 * @param  x_pwm: �ٶȣ�ռ�ձȣ�
 * @retval ��
 */
void Set_Left_Speed(uint16_t L_pwm)
{
  /* �ı�compareֵ�Ըı�ռ�ձ� */
  MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, L_pwm);
  Now_Left_Pwm = L_pwm;
}

void Set_Right_Speed(uint16_t R_pwm)
{
  /* �ı�compareֵ�Ըı�ռ�ձ� */
  MAP_Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, R_pwm);
  Now_Right_Pwm = R_pwm;
}

/**
 * @brief  ���õ������
 * @param  x_dir
 * @retval ��
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
 * @brief  �����������
 * @param  ��
 * @retval ��
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

/************************** ���ҵ�� ��������ٶ�pid���� *****************************/
/**
 * @brief  pid���� ·�̻�+�ٶȻ�
 * @param  ��
 * @retval ��
 */
void Speeds_Pid_Control()
{
  float temp_pwm = 0, temp_speed = 0, now_left_speed = 0, now_right_speed = 0;

  /* �����ٶ�pid */
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

  /* �ҵ���ٶ�pid */
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
  /* ��ӡ�������� */
  printf("%lf,%lf,%lf,%lf,%lf\r\n", (100 / (4.4 * 3.14)) * (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION), Left_journey_pulse, Right_journey_pulse, now_left_speed, now_right_speed);
  // printf("%lf,%lf,%lf,%lf,%lf\r\n", temp_speed, now_left_speed, now_right_speed);
}

/************************** ���ҵ�� �Ƕȿ����ٶ�pid���� *****************************/
/**
 * @brief  pid����
 * @param  ��
 * @retval ��
 */
void Angel_Pid_Control()
{
}

/************************** ���ҵ�� ����pid���� *****************************/
/**
 * @brief  pid���� ֻ���ٶȻ�
 * @param  ��
 * @retval ��
 */
void YunSu_Pid_Control()
{
  float temp_pwm = 0, now_left_speed = 0, now_right_speed = 0;

  /* ����pid */
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

  /* �ҵ��pid */
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
  /* ��ӡ�������� */
  // printf("%lf,%lf,%lf,%lf,%lf\r\n", (100 / (4.4 * 3.14)) * (REDUCTION_RATIO * ENCODER_TOTAL_RESOLUTION), Left_journey_pulse, Right_journey_pulse, now_left_speed, now_right_speed);
  // printf("%lf,%lf,%lf\r\n", pid_right_speed.target_val, now_left_speed, now_right_speed);
}

/*								������									*/
/*******************�����������ʼ��************************/
void Encoder_Configuration(void)
{
  /*����Ϊ������������*/
  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);
  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN7);
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);

  /*�����ⲿ�ж�*/
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);
  GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN7); // left motor  P1.6 | P1.7
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7); // right motor P2.6 | P2.7

  /*�����˿��ж�*/
  Interrupt_enableInterrupt(INT_PORT1);
  Interrupt_enableInterrupt(INT_PORT2);

  /*���ô�����ʽ*/
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
 *Description:�ⲿ�жϷ�����  ��¼�����ٶ�
 *Input:��
 *Output:��
 *Return:��
 *Others:�жϷ����������ж��ڴ������
 *****************************************************************/
void PORT1_IRQHandler(void)
{
  uint16_t flag;

  /*��ȡ�ж�״̬*/
  flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
  /*����жϱ�־λ*/
  GPIO_clearInterruptFlag(GPIO_PORT_P1, flag);
  /*���ֱ�������A*/
  if (flag & GPIO_PIN6)
  {
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == 0)
      Left_Once_Pulse++;
    else if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == 1)
      Left_Once_Pulse--;
  }

  /*���ֱ�������B*/
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
 *Description:�ⲿ�жϷ�����  ��¼�����ٶ�
 *Input:��
 *Output:��
 *Return:��
 *Others:�жϷ����������ж��ڴ������
 *****************************************************************/
void PORT2_IRQHandler(void)
{
  uint16_t flag;

  /*��ȡ�ж�״̬*/
  flag = GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
  /*����жϱ�־λ*/
  GPIO_clearInterruptFlag(GPIO_PORT_P2, flag);

  /*���ֱ�������A*/
  if (flag & GPIO_PIN7)
  {
    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == 0)
      Right_Once_Pulse++;
    else if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1)
      Right_Once_Pulse--;
  }

  /*���ֱ�������B*/
  if (flag & GPIO_PIN6)
  {
    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == 0)
      Right_Once_Pulse--;
    else if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1)
      Right_Once_Pulse++;
  }
}

/*******************ʵ������ʱ��ȡ��������ֵ************************/
void GetMotorPulse(void) //��ȡ�������
{
  Left_Total_Pulse += Left_Once_Pulse; //�ۼ�����	���һ��pid֮��Ҫ��0
  Right_Total_Pulse += Right_Once_Pulse;

  /* ת��ת�� = ��λʱ���ڵļ���ֵ / �������ֱܷ���=(����������ֱ���/4) * ʱ��ϵ��  */
  Motor_Shaft_Speed = (float)(Left_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));
 
  SendDataToLcd("Sysinfo.left_v.txt=\"%.2f\"", Motor_Shaft_Speed);
  // printf("���ת�ᴦת�٣�%.2f ת/�� \r\n", Motor_Shaft_Speed);
  // printf("��������ת�٣�%.2f ת/�� \r\n\n", Motor_Shaft_Speed / 20.0); /* �����ת�� = ת��ת�� / ���ٱ� */

  Motor_Shaft_Speed = (float)(Right_Once_Pulse * 1000 * 60) / (((ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO) * SPEED_PID_PERIOD));

  SendDataToLcd("Sysinfo.right_v.txt=\"%.2f\"", Motor_Shaft_Speed);
  // printf("���ת�ᴦת�٣�%.2f ת/�� \r\n", Motor_Shaft_Speed);
  // printf("��������ת�٣�%.2f ת/�� \r\n\n", Motor_Shaft_Speed / 20.0); /* �����ת�� = ת��ת�� / ���ٱ� */

  // printf("--------------------------------\r\n");

  Left_Once_Pulse = 0;
  Right_Once_Pulse = 0;
}
