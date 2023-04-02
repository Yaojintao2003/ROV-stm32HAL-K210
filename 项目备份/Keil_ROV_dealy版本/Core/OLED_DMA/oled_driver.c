/*
	ԭ������������:							MjGame 		https://space.bilibili.com/38673747
	ͬGifHub:								maoyongjie 	https://github.com/hello-myj/stm32_oled/
	��������ע��ɾ�������Ż��� ��������:	һֻ����Ե	https://space.bilibili.com/237304109
	����֮ǰ��ԭ�����汾����һͬ�ṩ,���������ַ��ȡ���������Ϣ,������������ԭ��ͬ��,��лԭ��
	
	ͼ�ο�ԭ��:��ʵ���Ƕ�һ��������в���,����������֮��,ֱ�ӽ�����
	����ˢ�µ���Ļ��
	��˴�c�ļ���������oled�ײ� ���ڵ�Ƭ����oled��ֱ����Ψһͨ��
	
	��ֲ��ͼ�ο���Ҫ�ı���������
	SPI_Configuration()	����ͨ������
	WriteCmd()			д����
	WriteDat()			д����
	OledTimeMsFunc()	oled_config�еĺ��� Ϊϵͳ�ṩʱ��
	�����̽�ΪSPIͨ�ŷ�ʽ ��Ҫ��������ͨ�ŷ�ʽ�ĺõײ�����3��������ž���
*/

#include "oled_driver.h"
#include "../Delay/Delay.h"
#include "draw_api.h"

extern unsigned char ScreenBuffer[SCREEN_PAGE_NUM][SCREEN_COLUMN];

uint8_t OLED_Init_CMD[] =
{
	0xAE, 	//--display off

	0x00, 	//---set low column address
	0x10, 	//---set high column address

	0x20,
	0x00,

	0x40, 	//--set start line address
	0xB0,	//--set page address

	0x81, 	// contract control
	0xFF, 	//--128

	0xA1, 	//set segment remap

	0xA6, 	//--normal / reverse

	0xA8,	//--set multiplex ratio(1 to 64)
	0x3F, 	//--1/32 duty

	0xC8, 	//Com scan direction

	0xD3, 	//-set display offset
	0x00-8,

	0xD5, 	//set osc division
	0x80,

	0xD8, 	//set area color mode off
	0x05,

	0xD9, 	//Set Pre-Charge Period
	0xF1,

	0xDA, 	//set com pin configuartion
	0x12,

	0xDB, 	//set Vcomh
	0x30,

	0x8D, 	//set charge pump enable
	0x14,

	0xAF	//--turn on oled panel
};

void OLED_FILL(unsigned char BMP[])
{
	HAL_I2C_Mem_Write_DMA(&Scr12864_HI2C, OLED_ADDRESS, OLED_WriteData_Addr, I2C_MEMADD_SIZE_8BIT, BMP, 1024);
}


void OLED_Init(void)
{
	HAL_I2C_Mem_Write_DMA(&Scr12864_HI2C, OLED_ADDRESS, OLED_WriteCom_Addr, I2C_MEMADD_SIZE_8BIT, OLED_Init_CMD, sizeof(OLED_Init_CMD));
	HAL_Delay(50);
	OLED_CLS();
	HAL_Delay(50);
}


void OLED_CLS(void)//���� ȫ������0x00
{
	unsigned char m,n;
	for(m=0;m<SCREEN_PAGE_NUM;m++)
	{
		for(n=0;n<SCREEN_COLUMN;n++)
		{
			ScreenBuffer[m][n] = 0;
		}
	}

	HAL_I2C_Mem_Write_DMA(&Scr12864_HI2C, OLED_ADDRESS, OLED_WriteData_Addr, I2C_MEMADD_SIZE_8BIT, ScreenBuffer[0], 1024);
}



