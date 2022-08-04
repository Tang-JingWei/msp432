#ifndef __MYIIC_H
#define __MYIIC_H
#include "sysinit.h" 
#include "stdio.h"


#define  SCL_PORT  GPIO_PORT_P6   // �ӣ�SCL��
#define  SCL_PIN   GPIO_PIN5      // �ӣ�SCL��
#define  SDA_PORT  GPIO_PORT_P6   // �ӣ�SDA��
#define  SDA_PIN   GPIO_PIN4      // �ӣ�SDA��

//IO��������
#define SDA_IN()        GPIO_setAsInputPin(SDA_PORT,SDA_PIN)	    //SDA����ģʽ
#define SDA_OUT()       GPIO_setAsOutputPin(SDA_PORT,SDA_PIN)     //SDA���ģʽ
//IO��������	 
#define IIC_SCL_High()  GPIO_setOutputHighOnPin(SCL_PORT,SCL_PIN) //SCL_High
#define IIC_SCL_Low()   GPIO_setOutputLowOnPin(SCL_PORT,SCL_PIN)  //SCL_Low
#define IIC_SDA_High()  GPIO_setOutputHighOnPin(SCL_PORT,SDA_PIN) //SDA_High
#define IIC_SDA_Low()   GPIO_setOutputLowOnPin(SCL_PORT,SDA_PIN)  //SDA_Low
#define READ_SDA        GPIO_getInputPinValue(SDA_PORT,SDA_PIN)   //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















