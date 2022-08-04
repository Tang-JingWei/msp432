#ifndef _BYYC_OLED_H_
#define _BYYC_OLED_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "sysinit.h"
	
#define PRI_BUF     300     // OLED��ʾ����ַ���
#define Brightness  0x0F    // OLED����
#define X_WIDTH     128     // OLED���
#define Y_WIDTH     64      // OLED�߶�

/*----- OLED�˿ڶ��� -----*/
/*  RES  */
#define RES_GPIO_Port  GPIO_PORT_P5
#define RES_GPIO_Pin   GPIO_PIN3
/*  DC  */
#define DC_GPIO_Port   GPIO_PORT_P5
#define DC_GPIO_Pin    GPIO_PIN2
/*  D0  */
#define D0_GPIO_Port   GPIO_PORT_P5
#define D0_GPIO_Pin    GPIO_PIN0
/*  D1  */
#define D1_GPIO_Port   GPIO_PORT_P5
#define D1_GPIO_Pin    GPIO_PIN1

#define OLED_RST_Clr()    GPIO_setOutputLowOnPin(RES_GPIO_Port, RES_GPIO_Pin); // RES����0Ӳ����λ��
#define OLED_RST_Set()    GPIO_setOutputHighOnPin(RES_GPIO_Port, RES_GPIO_Pin);  // RES

#define OLED_RS_Clr()     GPIO_setOutputLowOnPin(DC_GPIO_Port, DC_GPIO_Pin);    // DC����0Ϊд���
#define OLED_RS_Set()     GPIO_setOutputHighOnPin(DC_GPIO_Port, DC_GPIO_Pin);   // DC����1Ϊд���ݣ�

#define OLED_SCLK_Clr()   GPIO_setOutputLowOnPin(D0_GPIO_Port, D0_GPIO_Pin);    // D0��ʱ���ߣ���0����д�����ݣ�
#define OLED_SCLK_Set()   GPIO_setOutputHighOnPin(D0_GPIO_Port, D0_GPIO_Pin);   // D0��ʱ���ߣ���1д�����ݣ�������д�룩��

#define OLED_SDIN_Clr()   GPIO_setOutputLowOnPin(D1_GPIO_Port, D1_GPIO_Pin);    // D1�������ߣ�д0��
#define OLED_SDIN_Set()   GPIO_setOutputHighOnPin(D1_GPIO_Port, D1_GPIO_Pin);   // D1�������ߣ�д1��

#define OLED_CMD    0     //д����
#define OLED_DATA   1     //д����

extern unsigned char oled_picture[8][128];

void OLED_Init(void);
void OLED_Display_On(void);                                               //����OLED��ʾ
void OLED_Display_Off(void);                                              //�ر�OLED��ʾ
void OLED_Fill(unsigned char bmp_dat);                                    // OLED��䣨ÿҳ��ÿ��������bmp_dat����λ���ϣ�1Ϊ����
void OLED_CLS(void);                                                      // OLED��λ����OLED_Fill(0x00) ��������0��
void OLED_P6x8Char(unsigned char x, unsigned char y, unsigned char ch);   //��ʾ6*8һ����׼ASCII�ַ�,��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P6x8Str(unsigned char x, unsigned char y, unsigned char ch[]);  //��ʾ6*8һ���׼ASCII�ַ���,��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P8x16Char(unsigned char x, unsigned char y, unsigned char ch);  //��ʾ8*16��16<8��������Ҫ2ҳ��һ����׼ASCII�ַ�,��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P8x16Str(unsigned char x, unsigned char y, unsigned char ch[]); //��ʾ8*16һ���׼ASCII�ַ�����16<8��������Ҫ2ҳ������ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P16x16Ch(unsigned char x, unsigned char y, unsigned char n);    //��ʾ16*16������ʾ���֣�,��ʾ�����꣨x,y����yΪҳ��Χ0��7��NΪ��������
void OLED_my8x16Ch(uint8_t x, uint8_t y, uint8_t ch);                                    //��ʾ8*16һ���Լ��趨���֣�16<8��������Ҫ2ҳ������ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_Show_Position(uint8_t x, uint8_t y, const char *format, ...);             //�̶�λ����ʾ���ֻ��ַ�

void OLED_Show_Printf(uint8_t x, uint8_t y, uint8_t z, const char *format, ...);
void OLED_Draw_BMP(unsigned char BMP[][128]); //��ʾ��ʾBMPͼƬ128��64��ʼ������(0,0),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7

void OLEDPritnf(const char *format, ...);
void OLED_P6x8CharPutc(unsigned char *x, unsigned char *y, unsigned char ch);
void OLED_P8x16CharPutc(unsigned char *x, unsigned char *y, unsigned char ch);

#ifdef __cplusplus
}
#endif

#endif


