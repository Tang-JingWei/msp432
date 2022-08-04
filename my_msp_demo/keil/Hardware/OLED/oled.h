#ifndef _BYYC_OLED_H_
#define _BYYC_OLED_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "sysinit.h"
	
#define PRI_BUF     300     // OLED显示的最长字符串
#define Brightness  0x0F    // OLED亮度
#define X_WIDTH     128     // OLED宽度
#define Y_WIDTH     64      // OLED高度

/*----- OLED端口定义 -----*/
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

#define OLED_RST_Clr()    GPIO_setOutputLowOnPin(RES_GPIO_Port, RES_GPIO_Pin); // RES（置0硬件复位）
#define OLED_RST_Set()    GPIO_setOutputHighOnPin(RES_GPIO_Port, RES_GPIO_Pin);  // RES

#define OLED_RS_Clr()     GPIO_setOutputLowOnPin(DC_GPIO_Port, DC_GPIO_Pin);    // DC（置0为写命令）
#define OLED_RS_Set()     GPIO_setOutputHighOnPin(DC_GPIO_Port, DC_GPIO_Pin);   // DC（置1为写数据）

#define OLED_SCLK_Clr()   GPIO_setOutputLowOnPin(D0_GPIO_Port, D0_GPIO_Pin);    // D0（时间线，置0放入写的数据）
#define OLED_SCLK_Set()   GPIO_setOutputHighOnPin(D0_GPIO_Port, D0_GPIO_Pin);   // D0（时间线，置1写入数据（上升沿写入））

#define OLED_SDIN_Clr()   GPIO_setOutputLowOnPin(D1_GPIO_Port, D1_GPIO_Pin);    // D1（数据线，写0）
#define OLED_SDIN_Set()   GPIO_setOutputHighOnPin(D1_GPIO_Port, D1_GPIO_Pin);   // D1（数据线，写1）

#define OLED_CMD    0     //写命令
#define OLED_DATA   1     //写数据

extern unsigned char oled_picture[8][128];

void OLED_Init(void);
void OLED_Display_On(void);                                               //开启OLED显示
void OLED_Display_Off(void);                                              //关闭OLED显示
void OLED_Fill(unsigned char bmp_dat);                                    // OLED填充（每页，每竖行填满bmp_dat，低位在上，1为亮）
void OLED_CLS(void);                                                      // OLED复位（即OLED_Fill(0x00) （填满）0）
void OLED_P6x8Char(unsigned char x, unsigned char y, unsigned char ch);   //显示6*8一个标准ASCII字符,显示的坐标（x,y），y为页范围0～7
void OLED_P6x8Str(unsigned char x, unsigned char y, unsigned char ch[]);  //显示6*8一组标准ASCII字符串,显示的坐标（x,y），y为页范围0～7
void OLED_P8x16Char(unsigned char x, unsigned char y, unsigned char ch);  //显示8*16（16<8，所以需要2页）一个标准ASCII字符,显示的坐标（x,y），y为页范围0～7
void OLED_P8x16Str(unsigned char x, unsigned char y, unsigned char ch[]); //显示8*16一组标准ASCII字符串（16<8，所以需要2页），显示的坐标（x,y），y为页范围0～7
void OLED_P16x16Ch(unsigned char x, unsigned char y, unsigned char n);    //显示16*16点阵（显示汉字）,显示的坐标（x,y），y为页范围0～7，N为汉字索引
void OLED_my8x16Ch(uint8_t x, uint8_t y, uint8_t ch);                                    //显示8*16一组自己设定的字（16<8，所以需要2页），显示的坐标（x,y），y为页范围0～7
void OLED_Show_Position(uint8_t x, uint8_t y, const char *format, ...);             //固定位置显示数字或字符

void OLED_Show_Printf(uint8_t x, uint8_t y, uint8_t z, const char *format, ...);
void OLED_Draw_BMP(unsigned char BMP[][128]); //显示显示BMP图片128×64起始点坐标(0,0),x的范围0～127，y为页的范围0～7

void OLEDPritnf(const char *format, ...);
void OLED_P6x8CharPutc(unsigned char *x, unsigned char *y, unsigned char ch);
void OLED_P8x16CharPutc(unsigned char *x, unsigned char *y, unsigned char ch);

#ifdef __cplusplus
}
#endif

#endif


