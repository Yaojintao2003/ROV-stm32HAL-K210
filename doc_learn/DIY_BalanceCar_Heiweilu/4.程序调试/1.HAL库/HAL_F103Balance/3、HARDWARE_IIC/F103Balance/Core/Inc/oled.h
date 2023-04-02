#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"
#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC端口定义----------------  					   

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_RESET)//SCL
#define OLED_SCLK_Set() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_RESET)//SDA
#define OLED_SDIN_Set() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_SET)

 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(unsigned dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void SSD1306_Init(void);
void SSD1315_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
void OLED_ShowUnsignedNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p,uint8_t Char_Size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void Fill_Picture(unsigned char fill_Data);
void OLED_IIC_Start(void);
void OLED_IIC_Stop(void);
void OLED_Write_IIC_Command(unsigned char IIC_Command);
void OLED_Write_IIC_Data(unsigned char IIC_Data);
void OLED_Write_IIC_Byte(unsigned char IIC_Byte);
void OLED_IIC_Wait_Ack(void);

//void OLED_Operate_write(unsigned char x,unsigned char y,unsigned char asc);
void OLED_Float(unsigned char Y,unsigned char X,double real,unsigned char N);
//void OLED_Float2(unsigned char Y,unsigned char X,double real,unsigned char N1,unsigned char N2);
//void OLED_Num2(unsigned char x,unsigned char y, int number);
//void OLED_Num3(unsigned char x,unsigned char y,int number); 
//void OLED_Num4(unsigned char x,unsigned char y, int number);
//void OLED_Num5(unsigned char x,unsigned char y,unsigned int number);
void OLED_Show(void);
void OLED_Show_Str(void);
#endif
