#ifndef MOTOR
#define MOTOR


#include "sysinit.h"
#include "pid.h"

/* tb6612������� ����IO���� */
#define AIN1_GPIO_Port    GPIO_PORT_P4
#define AIN1_Pin          GPIO_PIN5
#define AIN2_GPIO_Port    GPIO_PORT_P4
#define AIN2_Pin          GPIO_PIN6
#define BIN1_GPIO_Port    GPIO_PORT_P4
#define BIN1_Pin          GPIO_PIN7
#define BIN2_GPIO_Port    GPIO_PORT_P9
#define BIN2_Pin          GPIO_PIN1

/* pwm��periodֵ */
#define PWM_PERIOD_COUNT     (2000)   

/* ����������ֱ��� */
#define ENCODER_RESOLUTION         13

/* ����2��Ƶ֮����ֱܷ��� */
#define ENCODER_TOTAL_RESOLUTION  (ENCODER_RESOLUTION * 2) 

/* ���ٵ�����ٱ� */
#define REDUCTION_RATIO 					 20

/*��sysTick�����PID��������ڣ��Ժ���Ϊ��λ*/
#define SPEED_PID_PERIOD  				 40    //���Ҫ����ʱ�����ж�����

/* ���ҵ������pwm���ڶ�ʱ�� */
#define Left_PWM_Interrupt 		htim3
#define Left_PWM_Channel   		TIM_CHANNEL_1

#define Right_PWM_Interrupt 	htim3
#define Right_PWM_Channel   	TIM_CHANNEL_2

/*  ���ڵ���ı��������ڶ�ʱ�� ��� */
#define Left_MOTOR_Interrupt 		htim1
#define Left_MOTOR_Timer   			TIM1

#define Right_MOTOR_Interrupt 	htim2
#define Right_MOTOR_Timer   		TIM2



/*  Motor ö�ٱ��� */

/* ת��Ƕ�ö�� */
typedef enum
{
  left_90,
	right_90,
	back_180
}spin_angle;

/* �������ö�� */
typedef enum
{
	MOTOR_STP = 0,
  MOTOR_FWD,
  MOTOR_BCK,
}motor_dir;


/********************/

extern int8_t 	Motor_Direction;
extern int32_t  Motor_Capture_Count;
extern float	  Motor_Shaft_Speed;

extern long 		Left_Once_Pulse;		//�����ֵ��ζ�ȡ������������
extern long 		Right_Once_Pulse;
extern long 		Left_Total_Pulse;		//�������ۼƻ��������
extern long 		Right_Total_Pulse;


/*  ��¼ ��� ����(DIR)|�ٶ�(PWM) --ȫ�ֵ���-- */
extern uint8_t  Now_Left_Dir;
extern uint8_t  Now_Right_Dir;
extern uint16_t Now_Left_Pwm;
extern uint16_t Now_Right_Pwm; 

extern uint8_t Line_flag,Turn_flag, Stop_flag;	    //����flag
extern uint8_t spin_start_flag , spin_succeed_flag; //ת��flag
extern uint8_t left_en_flag , right_en_flag;				//���ҵ��ʹ��flag

extern double Left_journey_pulse, Right_journey_pulse;	//���������߾��� , �о�����

/* ֱ��Ŀ�������� ��������ͬ */
extern double LineGo_Target_Maichong;		

/* ��ת��Ŀ�������� ������һ����һ����*/
extern double SpinLeft90_Target_Maichong;
extern double SpinRight90_Target_Maichong;
extern double SpinBack180_Target_Maichong;

extern float last_car_yaw;	//��¼ת��ǰʱ�̵ĽǶ�
extern float now_car_yaw;  //ת���ĵ�ǰ�Ƕ�
extern float target_yaw; //ת��Ŀ��Ƕ�

extern uint16_t stop_count, spin_count;	//pid����ĩβʱ����İڶ�����

/*  ��� PID ���  */
extern float L_Pwm_Outval, L_Location_Outval;	//����pid��Ŀ�����
extern float R_Pwm_Outval, R_Location2_Outval;

/* ��� IO����ʱ����ʼ�� */
void Motors_IO_Init(void);
void Motors_Timer_Init(void);

/*  ������  */
void Encoder_Configuration(void);
void GetMotorPulse(void);//��ȡ�������


/**********motor����**********/
void Motors_Control(motor_dir L_dir, uint16_t L_pwm, motor_dir R_dir, uint16_t R_pwm);
/* ��������ģʽ */
void Car_Go(int32_t desire_cm);
void Spin_Turn(spin_angle angle);
void YunSu_GO(float speed);
/* ����������� */
void Set_Left_Speed(uint16_t L_pwm);
void Set_Right_Speed(uint16_t R_pwm);
void Set_Left_Dir(motor_dir L_dir);
void Set_Right_Dir(motor_dir R_dir);
void Motor_L_forward(void);
void Motor_L_back(void);
void Motor_L_stop(void);
void Motor_R_forward(void);
void Motor_R_back(void);
void Motor_R_stop(void);


/*  pid  */
void Speeds_Pid_Control(void);	//���ҵ��ͬʱpid ·�̻�+�ٶȻ�
void YunSu_Pid_Control(void);   //���ҵ��ͬʱpid �ٶȻ�


#endif



