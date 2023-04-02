 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
#include "DataScope_DP.h"

/****************************��λ������*********************************/   
unsigned char i;          										 //��������
unsigned char Send_Count; 										 //������Ҫ���͵��ֽ���
/**********************************************************************/

int main(void)	
{ 
	float j;
	delay_init();	    	           //=====��ʱ������ʼ��	
	NVIC_Configuration();					 //=====�ж����ȼ�����
	uart1_init(128000);	           //=====����1��ʼ��
	LED_Init();                    //=====��ʼ���� LED ���ӵ�IO
	KEY_Init();                    //=====������ʼ��
	OLED_Init();                   //=====OLED��ʼ��
	OLED_Clear();									 //=====OLED����
  while(1)	
	{
			j+=0.1;
			if(j>3.14)  j=-3.14; 
			DataScope_Get_Channel_Data(10*sin(j), 1 );
			DataScope_Get_Channel_Data(10*cos(j), 2 );
			DataScope_Get_Channel_Data(2*j, 3 );
			Send_Count=DataScope_Data_Generate(3);
			for( i = 0 ; i < Send_Count; i++) 
			{
			while((USART1->SR&0X40)==0);  
			USART1->DR = DataScope_OutPut_Buffer[i]; 
			}
			delay_ms(50); //20HZ 
	} 	
}

