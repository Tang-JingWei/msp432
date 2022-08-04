#ifndef MOTOR
#define MOTOR


#include "sysinit.h"
#include "pid.h"

/* tb6612电机驱动 输入IO定义 */
#define AIN1_GPIO_Port    GPIO_PORT_P4
#define AIN1_Pin          GPIO_PIN5
#define AIN2_GPIO_Port    GPIO_PORT_P4
#define AIN2_Pin          GPIO_PIN6
#define BIN1_GPIO_Port    GPIO_PORT_P4
#define BIN1_Pin          GPIO_PIN7
#define BIN2_GPIO_Port    GPIO_PORT_P9
#define BIN2_Pin          GPIO_PIN1

/* pwm的period值 */
#define PWM_PERIOD_COUNT     (2000)   

/* 编码器物理分辨率 */
#define ENCODER_RESOLUTION         13

/* 经过2倍频之后的总分辨率 */
#define ENCODER_TOTAL_RESOLUTION  (ENCODER_RESOLUTION * 2) 

/* 减速电机减速比 */
#define REDUCTION_RATIO 					 20

/*在sysTick里调用PID计算的周期，以毫秒为单位*/
#define SPEED_PID_PERIOD  				 40    //这个要看定时器的中断周期

/* 左右电机控制pwm所在定时器 */
#define Left_PWM_Interrupt 		htim3
#define Left_PWM_Channel   		TIM_CHANNEL_1

#define Right_PWM_Interrupt 	htim3
#define Right_PWM_Channel   	TIM_CHANNEL_2

/*  用于电机的编码器所在定时器 相关 */
#define Left_MOTOR_Interrupt 		htim1
#define Left_MOTOR_Timer   			TIM1

#define Right_MOTOR_Interrupt 	htim2
#define Right_MOTOR_Timer   		TIM2



/*  Motor 枚举变量 */

/* 转弯角度枚举 */
typedef enum
{
  left_90,
	right_90,
	back_180
}spin_angle;

/* 电机方向枚举 */
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

extern long 		Left_Once_Pulse;		//左右轮单次读取编码器的脉冲
extern long 		Right_Once_Pulse;
extern long 		Left_Total_Pulse;		//左右轮累计获得脉冲数
extern long 		Right_Total_Pulse;


/*  记录 电机 方向(DIR)|速度(PWM) --全局调用-- */
extern uint8_t  Now_Left_Dir;
extern uint8_t  Now_Right_Dir;
extern uint16_t Now_Left_Pwm;
extern uint16_t Now_Right_Pwm; 

extern uint8_t Line_flag,Turn_flag, Stop_flag;	    //方向flag
extern uint8_t spin_start_flag , spin_succeed_flag; //转弯flag
extern uint8_t left_en_flag , right_en_flag;				//左右电机使能flag

extern double Left_journey_pulse, Right_journey_pulse;	//左右轮行走距离 , 感觉无用

/* 直行目标脉冲数 左右轮相同 */
extern double LineGo_Target_Maichong;		

/* 急转弯目标脉冲数 左右轮一个负一个正*/
extern double SpinLeft90_Target_Maichong;
extern double SpinRight90_Target_Maichong;
extern double SpinBack180_Target_Maichong;

extern float last_car_yaw;	//记录转弯前时刻的角度
extern float now_car_yaw;  //转弯后的当前角度
extern float target_yaw; //转弯目标角度

extern uint16_t stop_count, spin_count;	//pid调整末尾时允许的摆动次数

/*  电机 PID 相关  */
extern float L_Pwm_Outval, L_Location_Outval;	//经过pid后的控制量
extern float R_Pwm_Outval, R_Location2_Outval;

/* 电机 IO、定时器初始化 */
void Motors_IO_Init(void);
void Motors_Timer_Init(void);

/*  编码器  */
void Encoder_Configuration(void);
void GetMotorPulse(void);//读取电机脉冲


/**********motor控制**********/
void Motors_Control(motor_dir L_dir, uint16_t L_pwm, motor_dir R_dir, uint16_t R_pwm);
/* 三种运行模式 */
void Car_Go(int32_t desire_cm);
void Spin_Turn(spin_angle angle);
void YunSu_GO(float speed);
/* 电机基础控制 */
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
void Speeds_Pid_Control(void);	//左右电机同时pid 路程环+速度环
void YunSu_Pid_Control(void);   //左右电机同时pid 速度环


#endif



