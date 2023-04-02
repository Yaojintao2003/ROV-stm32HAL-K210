 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"

int	Encoder_Left,Encoder_Right; 

int main(void)	
{ 
	delay_init();	    	           //=====��ʱ������ʼ��	
	NVIC_Configuration();					 //=====�ж����ȼ�����
	uart1_init(9600);	             //=====����1��ʼ��
	LED_Init();                    //=====��ʼ���� LED ���ӵ�IO
	KEY_Init();                    //=====������ʼ��
	OLED_Init();                   //=====OLED��ʼ��
	OLED_Clear();									 //=====OLED����
	Encoder_Init_TIM2();					 //=====������2��ʼ��
	Encoder_Init_TIM4();					 //=====������4��ʼ��
	TIM3_Int_Init(99,7199);				 //=====10ms�ж�һ�Ρ����ȫ������ʱ���������ֵΪ90		
//	TIM3_Int_Init(49,7199);			   //=====5ms�ж�һ�Ρ����ȫ������ʱ���������ֵΪ45
  while(1)	
	{
			delay_ms(50);				 			 						//=====50msˢһ����Ļ����,����Ҫһֱˢ�¡�
			OLED_ShowString(0,0,"DAYUTC--OLED",12);
			OLED_ShowString(0,4,"L: ",12);
			OLED_ShowString(60,4,"R: ",12);
			OLED_Num2(4,4,Encoder_Left);					//��ʾ�ұߵ���ı�����ֵ
			OLED_Num2(14,4,Encoder_Right);				//��ʾ��ߵ���ı�����ֵ
			printf("Encoder_Left=%d,Encoder_Right=%d\n",Encoder_Left,Encoder_Right);
		  /*Ҳ����ʹ�õ�����λ���鿴,���û�й���OLED��Ļ�Ŀͻ��򿪵�����λ��,ʹ�ô��ڴ�ӡ����λ�����ɲ鿴***/
			LED=~LED;															//����������������				
	} 				
}
