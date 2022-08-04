#ifndef PID
#define PID

#include "sysinit.h"

/* pid������ �ṹ�� */
typedef struct
{
    float target_val;        //Ŀ��ֵ
    float actual_val;        //ʵ��ֵ
    float err;               //����ƫ��ֵ
    float err_last;          //������һ��ƫ��ֵ
    float Kp,Ki,Kd;          //������������֡�΢��ϵ��
    float integral;          //�������ֵ
}_pid;


//������pid�������ṹ��
extern _pid pid_left_speed, pid_right_speed;  
extern _pid pid_left_location, pid_right_location;  

void  PID_param_init(void);
void  set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void  set_p_i_d(_pid *pid, float p, float i, float d);

/* pid�㷨 */
float speed_pid_realize(_pid *pid, float actual_val);
float Location_pid_realize(_pid *pid, float actual_val);
float Angle_pid_realize(_pid *pid, float actual_val);


#endif



