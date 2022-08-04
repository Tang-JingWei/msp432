// OLED��ʾ
//��Ļ���ص�Ϊ128*64��ÿ8��Ϊһҳ����8ҳ
//��ʾa*b:aΪ�����ȣ�bΪ����߶ȣ���Ϊһҳ�ĸ߶�Ϊ8����b>8ʱ����Ҫb/8ҳ��
#include "oled.h"
#include "oledfont.h"

unsigned char oled_picture[8][128] = {0};

volatile uint8_t Flag_clear = 0;

// OLEDд����
void OLED_WrDat(unsigned char dat)
{
	uint8_t i;
	OLED_RS_Set(); // DC ��1д����
	for (i = 0; i < 8; i++)
	{
		OLED_SCLK_Clr();
		if (dat & 0x01)
		{
			OLED_SDIN_Set();
		}
		else
		{
			OLED_SDIN_Clr();
		}
		OLED_SCLK_Set();
		dat >>= 1;
	}
	OLED_RS_Set();
}

// OLEDд����
void OLED_WrCmd(unsigned char cmd)
{
	uint8_t i;
	OLED_RS_Clr(); // DC ��0д����
	for (i = 0; i < 8; i++)
	{
		OLED_SCLK_Clr();
		if (cmd & 0x80)
		{
			OLED_SDIN_Set();
		}
		else
		{
			OLED_SDIN_Clr();
		}
		OLED_SCLK_Set();
		cmd <<= 1;
	}
	OLED_RS_Set();
}

//����OLED��ʾ
void OLED_Display_On(void)
{
	OLED_WrCmd(0X8D); // SET DCDC�����ʼд��ɱã�
	OLED_WrCmd(0X14); // DCDC ON��bit2��1������ɱã�
	OLED_WrCmd(0XAF); // DISPLAY ON��������ʾ��
}

//�ر�OLED��ʾ
void OLED_Display_Off(void)
{
	OLED_WrCmd(0X8D); // SET DCDC�����ʼд��ɱã�
	OLED_WrCmd(0X10); // DCDC OFF��bit2��0�رյ�ɱã�
	OLED_WrCmd(0XAE); // DISPLAY OFF���ر���ʾ��
}

// OLED ��������
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WrCmd(0xb0 + (7 - y));			  //����ҳ��ַ
	OLED_WrCmd((x & 0x0f) | 0x00);		  //��ʼ��ַ��λ
	OLED_WrCmd(((x & 0xf0) >> 4) | 0x10); //��ʼ��ַ��λ
}

// OLED��䣨ÿҳ��ÿ��������bmp_dat����λ���ϣ�1Ϊ����
void OLED_Fill(unsigned char bmp_dat)
{
	unsigned char y, x;
	for (y = 0; y < 8; y++)
	{
		OLED_WrCmd(0xb0 + y); //ҳ��ַ(y���ַ)
		OLED_WrCmd(0x00);	  //�趨��ʼ��ַ��λ��x�ᣩ
		OLED_WrCmd(0x10);	  //�趨��ʼ��ַ��λ��x�ᣩ
		for (x = 0; x < X_WIDTH; x++)
			OLED_WrDat(bmp_dat);
	}
}

// OLED��λ����OLED_Fill(0x00) ��������0��
void OLED_CLS(void)
{
	unsigned char y, x;
	for (y = 0; y < 8; y++)
	{
		OLED_WrCmd(0xb0 + y); //ҳ��ַ(y���ַ)
		OLED_WrCmd(0x00);	  //�趨��ʼ��ַ��λ��x�ᣩ
		OLED_WrCmd(0x10);	  //�趨��ʼ��ַ��λ��x�ᣩ
		for (x = 0; x < X_WIDTH; x++)
			OLED_WrDat(0);
	}
}

//��ʾ6*8һ����׼ASCII�ַ�,��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P6x8Char(unsigned char x, unsigned char y, unsigned char ch)
{
	unsigned char c = 0, i;
	c = ch - 32;
	OLED_Set_Pos(x, y);		//���ó�ʼ����
	for (i = 0; i < 6; i++) // 6Ϊ������
		OLED_WrDat(F6x8[c][i]);
}

//��ʾ6*8һ���׼ASCII�ַ�������ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P6x8Str(unsigned char x, unsigned char y, unsigned char ch[])
{
	unsigned char c = 0, i = 0, j = 0;
	while (ch[j] != '\0') //�ж��ַ����Ƿ��ֹ
	{
		c = ch[j] - 32;
		if (x > 120) //��Ϊһҳ���Ϊ128���������һ��������ʼ�ڵ�128-int(128/6)*6=120��
		{
			x = 0;	   //�ص���1��
			y = y + 1; //����������һҳ
			if (y > 7)
			{
				break; //����һ����Ļ����ʾ��Χ��ֱ������
			}
		}

		OLED_Set_Pos(x, y); //���õ�һҳ��ʼ����
		for (i = 0; i < 6; i++)
			OLED_WrDat(F6x8[c][i]);
		OLED_Set_Pos(x, y + 1);
		x += 6; //�Զ�����6�У�������Ϊ6��
		j++;
	}
}

//��ʾ8*16��16<8��������Ҫ2ҳ��һ����׼ASCII�ַ�,��ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P8x16Char(unsigned char x, unsigned char y, unsigned char ch)
{
	unsigned char c = 0, i;
	c = ch - 32;
	OLED_Set_Pos(x, y);		//���õ�һҳ��ʼ����
	for (i = 0; i < 8; i++) // 8Ϊ������
		OLED_WrDat(F8X16[c][i]);
	OLED_Set_Pos(x, y + 1); //���õڶ�ҳ��ʼ����
	for (i = 0; i < 8; i++) // 8Ϊ������
		OLED_WrDat(F8X16[c][i + 8]);
}

//��ʾ8*16һ���׼ASCII�ַ�����16<8��������Ҫ2ҳ������ʾ�����꣨x,y����yΪҳ��Χ0��7
void OLED_P8x16Str(unsigned char x, unsigned char y, unsigned char ch[])
{
	unsigned char c = 0, i = 0, j = 0;
	while (ch[j] != '\0') //�ж��ַ����Ƿ��ֹ
	{
		c = ch[j] - 32;
		if (x > 120) //��Ϊһҳ���Ϊ128���������һ��������ʼ�ڵ�120��
		{
			x = 0;	   //�ص���1��
			y = y + 2; //������������ҳ����Ϊ�߶�Ϊ2������Ҫ����ҳ�������ʾ���У�
			if (y > 6)
			{
				break; //����һ����Ļ����ʾ��Χ��ֱ������
			}
		}

		OLED_Set_Pos(x, y); //���õ�һҳ��ʼ����
		for (i = 0; i < 8; i++)
			OLED_WrDat(F8X16[c][i]);
		OLED_Set_Pos(x, y + 1);
		for (i = 0; i < 8; i++)
			OLED_WrDat(F8X16[c][i + 8]);
		x += 8; //�Զ�����8��
		j++;
	}
}

//��ʼ��OLED
void OLED_Init(void)
{
	/*  һ��Ҫ����������ģʽ  */
	GPIO_setAsOutputPin(RES_GPIO_Port, RES_GPIO_Pin);
	GPIO_setAsOutputPin(DC_GPIO_Port, DC_GPIO_Pin);
	GPIO_setAsOutputPin(D0_GPIO_Port, D0_GPIO_Pin);
	GPIO_setAsOutputPin(D1_GPIO_Port, D1_GPIO_Pin);

	OLED_SCLK_Set();
	OLED_SDIN_Set();
	OLED_SDIN_Set();
	OLED_RST_Set();

	OLED_RST_Clr();	//Ӳ����λ
	delay_ms(100); //�ȴ���λ���
	OLED_RST_Set();
	OLED_WrCmd(0xAE); //�ر���ʾ
	OLED_WrCmd(0xD5); //����ʱ�ӷ�Ƶ����,��Ƶ��
	OLED_WrCmd(80);	  //[3:0],��Ƶ����;[7:4],��Ƶ��
	OLED_WrCmd(0xA8); //��������·��
	OLED_WrCmd(0X3F); //Ĭ��0X3F(1/64)
	OLED_WrCmd(0xD3); //������ʾƫ��
	OLED_WrCmd(0X00); //Ĭ��Ϊ0
	OLED_WrCmd(0x40); //������ʾ��ʼ�� [5:0],����.

	OLED_WrCmd(0x8D); //��ɱ�����
	OLED_WrCmd(0x14); // bit2������/�ر�
	OLED_WrCmd(0x20); //�����ڴ��ַģʽ
	OLED_WrCmd(0x02); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WrCmd(0xA1); //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WrCmd(0xC0); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WrCmd(0xDA); //����COMӲ����������
	OLED_WrCmd(0x12); //[5:4]����

	OLED_WrCmd(0x81); //�Աȶ�����
	OLED_WrCmd(0x5f); // 1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WrCmd(0xD9); //����Ԥ�������
	OLED_WrCmd(0xf1); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WrCmd(0xDB); //����VCOMH ��ѹ����
	OLED_WrCmd(0x30); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WrCmd(0xA4); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WrCmd(0xA6); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ
	OLED_WrCmd(0xAF); //������ʾ
	OLED_Fill(0x00);
	OLED_Set_Pos(0, 0);
}

unsigned char oled_overflowstop_flag = 0; //���ֹͣ��־λ��һ����Ļ��ʾ���£�
// OLEDPritnf(...)�����Ӻ�������ʾ6*8һ���׼ASCII�ַ���,��ʾ�����꣨x,y��
void OLED_P6x8CharPutc(unsigned char *x, unsigned char *y, unsigned char ch)
{
	unsigned char c = 0, i = 0;
	if (*x > 120) //�жϺ����Ƿ񳬳�����(120)��������������ỻ��
	{
		*x = 0;
		(*y) += 1;
	}
	if (*y > 7) //�ж������Ƿ񳬳�����(6)
	{
		*y = 0;
		oled_overflowstop_flag = 1; //��������ֹͣ��־λ
	}
	if (oled_overflowstop_flag == 0)
	{
		switch (ch)
		{
		case '\r': //�س�
		{
			*x = 0;
			return;
		}
		case '\n': //����
		{
			(*y) += 1;
			return;
		}
		case '\t': // TAB
		{
			(*x) += 24;
			return;
		}
		default: //ͨ���ַ���ʾ
		{
			c = ch - 32;
			OLED_Set_Pos(*x, *y);
			for (i = 0; i < 6; i++)
				OLED_WrDat(F6x8[c][i]);

			(*x) += 6;
			break;
		}
		}
	}
}

// OLEDPritnf(...)�����Ӻ�������ʾ8*16һ���׼ASCII�ַ�������ʾ�����꣨x,y��
void OLED_P8x16CharPutc(unsigned char *x, unsigned char *y, unsigned char ch)
{
	unsigned char c = 0, i = 0;
	if (*x > 120) //�жϺ����Ƿ񳬳�����(120)��������������ỻ��
	{
		*x = 0;
		(*y) += 2;
	}
	if (*y > 6) //�ж������Ƿ񳬳�����(6)���������������
	{
		*y = 0;
		oled_overflowstop_flag = 1; //��������ֹͣ��־λ
	}
	if (oled_overflowstop_flag == 0)
	{
		switch (ch)
		{
		case '\r': //�س�
		{
			*x = 0;
			return;
		}
		case '\n': //����
		{
			(*y) += 2;
			return;
		}
		case '\t': // TAB
		{
			(*x) += 32;
			return;
		}
		default: //ͨ���ַ���ʾ
		{
			c = ch - 32;
			OLED_Set_Pos(*x, *y);
			for (i = 0; i < 8; i++)
				OLED_WrDat(F8X16[c][i]);
			OLED_Set_Pos(*x, (*y) + 1);
			for (i = 0; i < 8; i++)
				OLED_WrDat(F8X16[c][i + 8]);
			(*x) += 8;
			break;
		}
		}
	}
}

// OLED��ʽ�����ʾ����
uint8_t OLED_buf[PRI_BUF];
void OLEDPritnf(const char *format, ...)
{
	uint16_t i, j;
	va_list ap;
	static unsigned char x = 0, y = 0;
	if (Flag_clear == 1)
	{
		x = 0;
		y = 0;
		OLED_Fill(0x00); //����
		Flag_clear = 0;
	}

	va_start(ap, format);
	vsprintf((char *)OLED_buf, format, ap);
	va_end(ap);

	i = strlen((const char *)OLED_buf); //�˴η������ݵĳ���
	for (j = 0; j < i; j++)				//ѭ����������
	{
		OLED_P6x8CharPutc(&x, &y, OLED_buf[j]);
	}
}

//�̶�λ����ʾ���ֻ��ַ�
void OLED_Show_Position(uint8_t x, uint8_t y, const char *format, ...)
{
	uint16_t i, j;
	va_list ap;
	va_start(ap, format);
	vsprintf((char *)OLED_buf, format, ap);
	va_end(ap);

	i = strlen((const char *)OLED_buf); //�˴η������ݵĳ���
	for (j = 0; j < i; j++)				//ѭ����������
	{
		if (oled_overflowstop_flag == 1)
		{
			oled_overflowstop_flag = 0; //���ֹͣ��־λ
			break;
		}
	  //OLED_P6x8CharPutc(&x, &y, OLED_buf[j]);
		OLED_P8x16CharPutc(&x, &y, OLED_buf[j]);
	}
}

//��ʾ��ʾBMPͼƬ128��64��ʼ������(0,0),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7��ͼ����ʾ�Ѿ���ȫ��ɨ�����Բ���ÿһ�ζ����и�λ����Ȼ������
void OLED_Draw_BMP(unsigned char BMP[][128])
{
	unsigned char x, y;

	for (y = 0; y < 64; y++)
	{
		for (x = 0; x < 128; x++)
		{
			OLED_Set_Pos(x, y);	   //����ÿ8���ŵ�ĳ�ʼ����
			OLED_WrDat(BMP[y][x]); //��ʾÿһ��
		}
	}
}

// void mode_oled_show()
//{
//	Uc_Array2_All_Num((unsigned char *)oled_picture, 8, 128, 0); //����

//	OLED_Show_Printf(0, 0, 1, "motor direction:");
//	OLED_Show_Printf(2, 0, 1, "%s", forward_word);
//	OLED_Show_Printf(4, 0, 1, "right value: %d", 100);
//	OLED_Show_Printf(6, 0, 1, "left  value: %d", 100);

//	OLED_Draw_BMP(oled_picture); //ˢ��
//}
