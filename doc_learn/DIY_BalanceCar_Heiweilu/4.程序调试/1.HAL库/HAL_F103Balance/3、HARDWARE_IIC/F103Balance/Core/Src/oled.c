//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//�о�԰����
//���̵�ַ��http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  �� �� ��   : main.c
//  �� �� ��   : v2.0
//  ��    ��   : Evk123
//  ��������   : 2014-0101
//  ����޸�   : 
//  ��������   : 0.69��OLED �ӿ���ʾ����(STM32F103ZEϵ��IIC)
//              ˵��: 
//              ----------------------------------------------------------------
//              GND   ��Դ��
//              VCC   ��5V��3.3v��Դ
//              SCL   ��PB13��SCL��
//              SDA   ��PB15��SDA�� 
//              ----------------------------------------------------------------
//Copyright(C) �о�԰����2014/3/16
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

#include "oled.h"
#include "oledfont.h"  	 

//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   
/**********************************************
//IIC Start
**********************************************/
void OLED_IIC_Start()
{

	OLED_SCLK_Set() ;
	OLED_SDIN_Set();
	OLED_SDIN_Clr();
	OLED_SCLK_Clr();
}

/**********************************************
//IIC Stop
**********************************************/
void OLED_IIC_Stop()
{
	OLED_SCLK_Set() ;
//	OLED_SCLK_Clr();
	OLED_SDIN_Clr();
	OLED_SDIN_Set();
	
}

void OLED_IIC_Wait_Ack()
{
	OLED_SCLK_Set() ;
	OLED_SCLK_Clr();
}
/**********************************************
// IIC Write byte
**********************************************/

void OLED_Write_IIC_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	unsigned char m,da;
	da=IIC_Byte;
	OLED_SCLK_Clr();
	for(i=0;i<8;i++)		
	{
		m=da;
		//	OLED_SCLK_Clr();
		m=m&0x80;
		if(m==0x80)
		{OLED_SDIN_Set();}
		else OLED_SDIN_Clr();
		da=da<<1;
		OLED_SCLK_Set();
		OLED_SCLK_Clr();
	}
}
/**********************************************
// IIC Write Command
**********************************************/
void OLED_Write_IIC_Command(unsigned char IIC_Command)
{
		OLED_IIC_Start();
		OLED_Write_IIC_Byte(0x78);            //Slave address,SA0=0
		OLED_IIC_Wait_Ack();	
		OLED_Write_IIC_Byte(0x00);			//write command
		OLED_IIC_Wait_Ack();	
		OLED_Write_IIC_Byte(IIC_Command); 
		OLED_IIC_Wait_Ack();	
		OLED_IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void OLED_Write_IIC_Data(unsigned char IIC_Data)
{
		OLED_IIC_Start();
		OLED_Write_IIC_Byte(0x78);			//D/C#=0; R/W#=0
		OLED_IIC_Wait_Ack();	
		OLED_Write_IIC_Byte(0x40);			//write data
		OLED_IIC_Wait_Ack();	
		OLED_Write_IIC_Byte(IIC_Data);
		OLED_IIC_Wait_Ack();	
		OLED_IIC_Stop();
}
void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
	{
   OLED_Write_IIC_Data(dat);
	}
	else
	{
   OLED_Write_IIC_Command(dat);
	}
}


/********************************************
// Fill_Picture
********************************************/
void Fill_Picture(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);		//page0-page1
		OLED_WR_Byte(0x00,0);		//low column start address
		OLED_WR_Byte(0x10,0);		//high column start address
		for(n=0;n<128;n++)
			{
				OLED_WR_Byte(fill_Data,1);
			}
	}
}

//��������

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //������ʾ
}
void OLED_On(void)
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //������ʾ
}
//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//�õ�ƫ�ƺ��ֵ			
		if(x>Max_Column-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else {	
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_Byte(F6x8[c][i],OLED_DATA);
				
			}
}
//m^n����
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
void OLED_ShowNum(uint8_t x,uint8_t y,int num,uint8_t len,uint8_t size2)
{
	if(num<0)
	{
		OLED_ShowString(x,y,"-",size2); 
		num=-num;
	}
	else	
	{
		OLED_ShowString(x,y,"+",size2); 
	}
	OLED_ShowUnsignedNum(x+size2/2,y,num,len,size2);
}
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
void OLED_ShowUnsignedNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size2)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
//��ʾһ���ַ��Ŵ�
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j],Char_Size);
			x+=8;
		if(x>120)break;
			j++;
	}
}
//��ʾ����
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	uint8_t t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
				OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
				adder+=1;
      }					
}
/***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
	    }
	}
}

void OLED_Float(unsigned char Y,unsigned char X,double real,unsigned char N) 
{
   unsigned char   i_Count=1;
   unsigned char   n[12]={0};
   long   j=1;  
   int    real_int=0;
   double decimal=0;
   unsigned int   real_decimal=0;
   if(real<0)
	 {
		 real_int=(int)(-real);
	 }
	 else
	 {
		 real_int=(int)real;
   }
	 decimal=real-real_int;
   real_decimal=decimal*1e4;
   while(real_int/10/j!=0)
   {
      j=j*10;i_Count++;  
   } 
   n[0]=(real_int/10000)%10; 
   n[1]=(real_int/1000)%10;
   n[2]=(real_int/100)%10;
   n[3]=(real_int/10)%10;
   n[4]=(real_int/1)%10; 
   n[5]='.';
   n[6]=(real_decimal/1000)%10;
   n[7]=(real_decimal/100)%10;
   n[8]=(real_decimal/10)%10;
   n[9]=real_decimal%10;
   n[6+N]='\0'; 
   for(j=0;j<10;j++) n[j]=n[j]+16+32;
	 if(real<0) 
	 {		 
		 i_Count+=1;
		 n[5-i_Count]='-';
	 }
   n[5]='.';
   n[6+N]='\0';   
   OLED_ShowString(X,Y,&n[5-i_Count],12); 
}

//��ʼ��SSD1306					    
void SSD1306_Init(void)
{
	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//
	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//
	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//
	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//
	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//
	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//
	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//
	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
}  
void  SSD1315_Init(void)
{
	OLED_WR_Byte(0xAE,OLED_CMD);    /*display off*/      

	OLED_WR_Byte(0x00,OLED_CMD);    /*set lower column address*/       
	OLED_WR_Byte(0x10,OLED_CMD);    /*set higher column address*/

	OLED_WR_Byte(0x40,OLED_CMD);    /*set display start line*/

	OLED_WR_Byte(0xB0,OLED_CMD);    /*set page address*/

	OLED_WR_Byte(0x81,OLED_CMD);    /*contract control*/
	OLED_WR_Byte(0xb0,OLED_CMD);    /*128*/

	OLED_WR_Byte(0xA1,OLED_CMD);    /*set segment remap*/

	OLED_WR_Byte(0xA4,OLED_CMD);    

	OLED_WR_Byte(0xA6,OLED_CMD);    /*normal / reverse*/

	OLED_WR_Byte(0xA8,OLED_CMD);    /*multiplex ratio*/
	OLED_WR_Byte(0x3F,OLED_CMD);    /*duty = 1/64*/

	OLED_WR_Byte(0xC8,OLED_CMD);    /*Com scan direction*/

	OLED_WR_Byte(0xD3,OLED_CMD);    /*set display offset*/
	OLED_WR_Byte(0x00,OLED_CMD);

	OLED_WR_Byte(0xD5,OLED_CMD);    /*set osc division*/
	OLED_WR_Byte(0x90,OLED_CMD);    

	OLED_WR_Byte(0xD9,OLED_CMD);    /*set pre-charge period*/
	OLED_WR_Byte(0x22,OLED_CMD);

	OLED_WR_Byte(0xDA,OLED_CMD);    /*set COM pins*/
	OLED_WR_Byte(0x12,OLED_CMD);

	OLED_WR_Byte(0xdb,OLED_CMD);    /*set vcomh*/
	OLED_WR_Byte(0x30,OLED_CMD);

	OLED_WR_Byte(0x8d,OLED_CMD);    /*set charge pump enable*/
	OLED_WR_Byte(0x14,OLED_CMD);

	OLED_WR_Byte(0xAF,OLED_CMD);    /*display ON*/                           

}
void OLED_Show_Str(void)
{	
	OLED_ShowString(0,0,"ANGLE:",12);
	OLED_ShowString(0,1,"BAT :",12);
	OLED_ShowString(90,1,"V",12);
	OLED_ShowString(0,2,"Distance:",12);
	OLED_ShowString(60,3,"R: ",12);	
	OLED_ShowString(1,3,"L: ",12);
}

void OLED_Show(void)
{
		OLED_Float(0,56,pitch,2);	
		OLED_Float(1,48,BatteryVoltage,2);						//��ʾ��ѹ
		OLED_ShowNum(80,3,Encoder_Right,3,12);					//��ʾ�ұߵ���ı�����ֵ
		OLED_ShowNum(20,3,Encoder_Left,3,12);					//��ʾ��ߵ���ı�����ֵ
		OLED_Float(2,70,1000,2);						//��ʾ�������ľ���
}
