#include "graysensor.h"

/*  ��������־λ  */
uint8_t L2_flag, L1_flag, MID_flag, R1_flag, R2_flag;

/*  Ѳ�߲�������  */
float Search_Buchang = 0;


/* �Ҷȴ����� IO��ʼ�� */
void Gray_Init(void) 
{ 
  GPIO_setAsInputPin(GRAY_L1_GPIO_Port,GRAY_L1_Pin);
  GPIO_setAsInputPin(GRAY_L1_GPIO_Port, GRAY_L1_Pin);
  GPIO_setAsInputPin(GRAY_L2_GPIO_Port, GRAY_L2_Pin);
  GPIO_setAsInputPin(GRAY_MID_GPIO_Port,GRAY_MID_Pin);
  GPIO_setAsInputPin(GRAY_R1_GPIO_Port, GRAY_R1_Pin); 
  GPIO_setAsInputPin(GRAY_R2_GPIO_Port, GRAY_R2_Pin); 
}


/*  
�Ҷȴ�����ѭ�� ����0/1  

�Ҷ�ֵ�ϸߣ��ڣ��������ø�	flag = 1
�Ҷ�ֵ�ϵͣ��ף��������õ�	flag = 0
*/
void Gray_Search(void)  
{
	
	if(GPIO_getInputPinValue(GRAY_L1_GPIO_Port, GRAY_L1_Pin) == 1){L2_flag = 1;} else {L2_flag = 0;}
	if(GPIO_getInputPinValue(GRAY_L2_GPIO_Port, GRAY_L2_Pin) == 1){L1_flag = 1;} else {L1_flag = 0;}
	if(GPIO_getInputPinValue(GRAY_MID_GPIO_Port,GRAY_MID_Pin) == 1){MID_flag = 1;} else {MID_flag = 0;}
	if(GPIO_getInputPinValue(GRAY_R1_GPIO_Port, GRAY_R1_Pin) == 1){R1_flag = 1;} else {R1_flag = 0;}
	if(GPIO_getInputPinValue(GRAY_R2_GPIO_Port, GRAY_R2_Pin) == 1){R2_flag = 1;} else {R2_flag = 0;}
	
	/*  Ѱ�����岹��pwm  */
	if(L1_flag == 0 &&  MID_flag == 1 && R1_flag == 0) Search_Buchang =  0;
	else if(L1_flag == 0 &&  MID_flag == 1 && R1_flag == 1) Search_Buchang =  700;
	else if(L1_flag == 0 &&  MID_flag == 0 && R1_flag == 1) Search_Buchang =  600;
	else if(L1_flag == 1 &&  MID_flag == 0 && R1_flag == 0) Search_Buchang = -600;   
  else if(L1_flag == 1 &&  MID_flag == 1 && R1_flag == 0) Search_Buchang = -700;	
	else if(L2_flag == 1 &&  L1_flag == 0 &&  MID_flag == 0 && R1_flag == 0 && R2_flag == 0) Search_Buchang = -1000;	
  else if(L2_flag == 0 &&  L1_flag == 0 &&  MID_flag == 0 && R1_flag == 0 && R2_flag == 1) Search_Buchang = 1000;
	else if(L2_flag == 0 &&  L1_flag == 0 &&  MID_flag == 0 && R1_flag == 0 && R2_flag == 0) Search_Buchang = 0;	
	else Search_Buchang =  0;
	
	
}



