#include "control.h"	
 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��	
				 ��MPU6050�Ĳ���Ƶ�������У����ó�100HZ�����ɱ�֤6050��������10ms����һ�Ρ�
				 ���߿���imv_mpu.h�ļ���26�еĺ궨������޸�(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

float Mechanical_angle=0; // �޳����������ƽ���ŵ�С����е��ֵ

float balance_UP_KP=1000; 	 // С��ֱ����KP(��һ��kp��1000 ����������0.6       1000    //600
float balance_UP_KD=-1.2;              //-1.2                                    -1.2                          //-1.08
float velocity_KP=0;      // 1.0
float velocity_KI=0;    //0.005

void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)		
	{		
		EXTI->PR=1<<5;                                           //===���LINE5�ϵ��жϱ�־λ   
		mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===�õ�ŷ���ǣ���̬�ǣ�������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===�õ�����������
		Encoder_Left=-Read_Encoder(2);                           //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		Encoder_Right=Read_Encoder(4);                           //===��ȡ��������ֵ
		Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===ֱ����PID����	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);       //===�ٶȻ�PID����	 
		//Turn_Pwm=(-0.5)*gyroz;																	 //===����У��С����ԭ��תȦ������
		Moto1=Balance_Pwm-Velocity_Pwm-Turn_Pwm  ;                 //===�������ֵ������PWM
		Moto2=Balance_Pwm-Velocity_Pwm+Turn_Pwm  ;            		//===�������ֵ������PWM	
	  Xianfu_Pwm();  																					 //===PWM�޷�
		Turn_Off(pitch);																         //===���Ƕ��Ƿ�����
		Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���  
	}       	
} 

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ���еƽ��Ƕȣ���е��ֵ�������ٶ�
����  ֵ��ֱ������PWM
��    �ߣ��������
**************************************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 //===���ƽ��ĽǶ���ֵ�ͻ�е���
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ����������������ֵ
����  ֵ���ٶȿ���PWM
��    �ߣ��������
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
   //=============�ٶ�PI������=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  		Encoder_Integral=10000;             //===�����޷�
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===�����޷�	
		Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===�ٶȿ���	
	  if(pitch<-40||pitch>40) 				Encoder_Integral=0;     						//===����رպ��������
	  return Velocity;
}
