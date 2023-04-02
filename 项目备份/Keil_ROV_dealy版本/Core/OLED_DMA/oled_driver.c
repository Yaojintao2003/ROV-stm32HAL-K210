/*
	原作者哔哩哔哩:							MjGame 		https://space.bilibili.com/38673747
	同GifHub:								maoyongjie 	https://github.com/hello-myj/stm32_oled/
	代码整理注释删减增加优化等 哔哩哔哩:	一只程序缘	https://space.bilibili.com/237304109
	整理之前的原代码随本代码一同提供,浏览以上网址获取更多相关信息,本代码以征得原作同意,感谢原作
	
	图形库原理:其实就是对一个数组进行操作,数组操作完成之后,直接将整个
	数组刷新到屏幕上
	因此此c文件用于配置oled底层 用于单片机与oled的直接且唯一通信
	
	移植此图形库主要改变以下内容
	SPI_Configuration()	配置通信引脚
	WriteCmd()			写命令
	WriteDat()			写数据
	OledTimeMsFunc()	oled_config中的函数 为系统提供时基
	此例程仅为SPI通信方式 需要更改其他通信方式改好底层上面3个函数大概就行
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


void OLED_CLS(void)//清屏 全部发送0x00
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



