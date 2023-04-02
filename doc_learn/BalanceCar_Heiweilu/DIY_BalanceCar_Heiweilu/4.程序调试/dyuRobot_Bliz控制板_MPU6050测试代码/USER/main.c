 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
#include "DataScope_DP.h"

//////////////////////////ȫ�ֱ����Ķ���////////////////////////////////////  
float pitch,roll,yaw; 								  			 //ŷ����(��̬��)
short aacx,aacy,aacz;													 //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;											 //������ԭʼ����
float temp; 								  								 //�¶�

unsigned char i;          										 //��������
unsigned char Send_Count; 										 //������Ҫ���͵��ֽ���
float j;
////////////////////////////////////////////////////////////////////////////
int main(void)	
{ 
	delay_init();	    	           //=====��ʱ������ʼ��	
	NVIC_Configuration();					 //=====�ж����ȼ�����,���а��������е��ж����ȼ�������,��������һ�����޸ġ�
	LED_Init();                    //=====��ʼ���� LED ���ӵ�IO
	KEY_Init();                    //=====������ʼ��
	uart1_init(128000);	             //=====����1��ʼ��
	TIM3_Init(99,7199);	    		   //=====��ʱ����ʼ�� 100ms�ж�һ��
	OLED_Init();                   //=====OLED��ʼ��
	OLED_Clear();									 //=====OLED����
	MPU_Init();					    			 //=====��ʼ��MPU6050
	mpu_dmp_init();								 //=====��ʼ��MPU6050��DMPģʽ	

//	OLED_ShowString(0,0,"Pitch:",12);
//	OLED_ShowString(0,2,"Roll :",12);
//	OLED_ShowString(0,4,"Yaw  :",12);
//	OLED_ShowString(0,6,"Temp :",12);
//	
 




while(1)	
	{
		/*�������ǲ鿴���ݵı仯,�鿴��̬��;�����Ҫ�����ٶ��Լ����ٶȿ������޸�***/
		mpu_dmp_get_data(&pitch,&roll,&yaw);			//�õ���̬�Ǽ�ŷ����
		temp=MPU_Get_Temperature();								//�õ��¶�ֵ
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������	
//		OLED_Float(0,48,pitch,2);									//��ʾ������
//		OLED_Float(2,48,roll,2);									//��ʾ������
//		OLED_Float(4,48,yaw,2);										//��ʾ�����
//		OLED_Float(6,48,temp/100,2);							//��ʾ�¶�
		printf("pitch=%.2f,roll=%.2f,yaw=%.2f,temp=%.2f\n",pitch,roll,yaw,temp/100);
		/*Ҳ����ʹ�õ�����λ���鿴,���û�й���OLED��Ļ�Ŀͻ��򿪵�����λ��,ʹ�ô��ڴ�ӡ����λ�����ɲ鿴***/
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













