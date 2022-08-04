#include "pid.h"

//������pid�������ṹ��
_pid pid_left_speed, pid_right_speed;
_pid pid_left_location, pid_right_location;

/**
 * @brief  PID������ʼ��
 *	@note 	��
 * @retval ��
 */
void PID_param_init()
{
  /************* Location **************/
  /* left pid�ṹ���ʼ�� */
  pid_left_location.target_val = 0.0;
  pid_left_location.actual_val = 0.0;
  pid_left_location.err = 0.0;
  pid_left_location.err_last = 0.0;
  pid_left_location.integral = 0.0;

  /* pid ���� */
  pid_left_location.Kp = 0.13;
  pid_left_location.Ki = 0.003;
  pid_left_location.Kd = 0.002;

  /* right pid�ṹ���ʼ�� */
  pid_right_location.target_val = 0.0;
  pid_right_location.actual_val = 0.0;
  pid_right_location.err = 0.0;
  pid_right_location.err_last = 0.0;
  pid_right_location.integral = 0.0;

  /* pid ���� */
  pid_right_location.Kp = 0.13;
  pid_right_location.Ki = 0.003;
  pid_right_location.Kd = 0.002;

  /************** Speed *************/
  /* left pid�ṹ���ʼ�� */
  pid_left_speed.target_val = 0.0;
  pid_left_speed.actual_val = 0.0;
  pid_left_speed.err = 0.0;
  pid_left_speed.err_last = 0.0;
  pid_left_speed.integral = 0.0;

  /* pid ���� */
  pid_left_speed.Kp = 4;
  pid_left_speed.Ki = 0.01;
  pid_left_speed.Kd = 0.00;

  /* right pid�ṹ���ʼ�� */
  pid_right_speed.target_val = 0.0;
  pid_right_speed.actual_val = 0.0;
  pid_right_speed.err = 0.0;
  pid_right_speed.err_last = 0.0;
  pid_right_speed.integral = 0.0;

  /* pid ���� */
  pid_right_speed.Kp = 4;
  pid_right_speed.Ki = 0.01;
  pid_right_speed.Kd = 0.00;
}

/**
 * @brief  ����Ŀ��ֵ
 * @param  val		Ŀ��ֵ
 *	@note 	��
 * @retval ��
 */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val; // ���õ�ǰ��Ŀ��ֵ
}

/**
 * @brief  ��ȡĿ��ֵ
 * @param  ��
 *	@note 	��
 * @retval Ŀ��ֵ
 */
float get_pid_target(_pid *pid)
{
  return pid->target_val; // ���õ�ǰ��Ŀ��ֵ
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  pid->Kp = p; // ���ñ���ϵ�� P
  pid->Ki = i; // ���û���ϵ�� I
  pid->Kd = d; // ����΢��ϵ�� D
}

/**
 * @brief  ����ʵ���ٶȻ������pwm PID�㷨ʵ��
 * @param  actual_val:ʵ��ֵ
 *	@note 	��
 * @retval ͨ��PID��������� pwm
 */
float speed_pid_realize(_pid *pid, float actual_val)
{
  /*����Ŀ��ֵ��ʵ��ֵ�����*/
  pid->err = pid->target_val - actual_val;

  if ((pid->err < 1) && (pid->err > -1)) 
  {
    pid->err = 0.0f;
  }

  pid->integral += pid->err; // ����ۻ�

  /*�����޷�*/
  if (pid->integral >= 20000)
  {
    pid->integral = 20000;
  }
  else if (pid->integral < -20000)
  {
    pid->integral = -20000;
  }

  /*PID�㷨ʵ��*/
  pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  /*����*/
  pid->err_last = pid->err;

  /*���ص�ǰʵ��ֵ*/
  return pid->actual_val;
}

/**
 * @brief  ����ʵ��·�̻�������ٶ�  PID�㷨ʵ��
 * @param  actual_val:ʵ��ֵ
 *	@note 	��
 * @retval ͨ��PID��������� �ٶ�
 */
float Location_pid_realize(_pid *pid, float actual_val)
{
  /*����Ŀ��ֵ��ʵ��ֵ�����*/
  pid->err = pid->target_val - actual_val;

  if ((pid->err < 10) && (pid->err > -10)) 
  {
    pid->err = 0.0f;
  }

  pid->integral += pid->err; // ����ۻ�

  /*�����޷�*/
  if (pid->integral >= 15000)
  {
    pid->integral = 15000;
  }
  else if (pid->integral < -15000)
  {
    pid->integral = -15000;
  }

  /*PID�㷨ʵ��*/
  pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  /*����*/
  pid->err_last = pid->err;

  /* �ٶ����޴��� */
  if ((pid->actual_val) > 350)
  {
    pid->actual_val = 350;
  }
  else if ((pid->actual_val) < -350)
  {
    pid->actual_val = -350;
  }

  /*���ص�ǰʵ��ֵ*/
  return pid->actual_val;
}

/**
 * @brief  ����ʵ�ʽǶȻ�������ٶ�  PID�㷨ʵ��
 * @param  actual_val:ʵ��ֵ
 *	@note 	��
 * @retval ͨ��PID��������� �ٶ�
 */
float Angle_pid_realize(_pid *pid, float actual_val)
{
  /*����Ŀ��ֵ��ʵ��ֵ�����*/
  pid->err = pid->target_val - actual_val;

  if ((pid->err < 10) && (pid->err > -10)) 
  {
    pid->err = 0.0f;
  }

  pid->integral += pid->err; // ����ۻ�

  /*�����޷�*/
  if (pid->integral >= 15000)
  {
    pid->integral = 15000;
  }
  else if (pid->integral < -15000)
  {
    pid->integral = -15000;
  }

  /*PID�㷨ʵ��*/
  pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

  /*����*/
  pid->err_last = pid->err;

  /* �ٶ����޴��� */
  if ((pid->actual_val) > 350)
  {
    pid->actual_val = 350;
  }
  else if ((pid->actual_val) < -350)
  {
    pid->actual_val = -350;
  }

  /*���ص�ǰʵ��ֵ*/
  return pid->actual_val;
}
