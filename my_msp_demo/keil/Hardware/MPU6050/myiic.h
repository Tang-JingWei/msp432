#ifndef __MYIIC_H
#define __MYIIC_H
#include "sysinit.h" 
#include "stdio.h"


#define  SCL_PORT  GPIO_PORT_P6   // 接（SCL）
#define  SCL_PIN   GPIO_PIN5      // 接（SCL）
#define  SDA_PORT  GPIO_PORT_P6   // 接（SDA）
#define  SDA_PIN   GPIO_PIN4      // 接（SDA）

//IO方向设置
#define SDA_IN()        GPIO_setAsInputPin(SDA_PORT,SDA_PIN)	    //SDA输入模式
#define SDA_OUT()       GPIO_setAsOutputPin(SDA_PORT,SDA_PIN)     //SDA输出模式
//IO操作函数	 
#define IIC_SCL_High()  GPIO_setOutputHighOnPin(SCL_PORT,SCL_PIN) //SCL_High
#define IIC_SCL_Low()   GPIO_setOutputLowOnPin(SCL_PORT,SCL_PIN)  //SCL_Low
#define IIC_SDA_High()  GPIO_setOutputHighOnPin(SCL_PORT,SDA_PIN) //SDA_High
#define IIC_SDA_Low()   GPIO_setOutputLowOnPin(SCL_PORT,SDA_PIN)  //SDA_Low
#define READ_SDA        GPIO_getInputPinValue(SDA_PORT,SDA_PIN)   //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















