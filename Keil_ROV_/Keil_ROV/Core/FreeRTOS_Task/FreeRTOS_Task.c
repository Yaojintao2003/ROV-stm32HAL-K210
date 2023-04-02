/*
 * FreeRTOS_Task.c
 *
 *  Created on: Nov 28, 2020
 *      Author: Royic
 */
#include "main.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "../MS5837/ms5837.h"
#include "../OLED_DMA/draw_api.h"
#include "../OLED_DMA/oled_myapp.h"
#include "../ESC/ESC.h"
#include "../UART_DMA/UART_DMA.h"
#include "../Kalman/Kalman.h"
#include "../PID/PID.h"
#include <math.h>
#include "../WS2812/WS2812.h"
#include "../Range2Percentage/Range2Percentage.h" 

#include "../MPU6050/mpu6050.h"
#include "../MPU6050/eMPL/inv_mpu.h"
#include "../kalman/kalman.h"

extern pid_type_def Yaw_PID;

float Pitch, Roll, Yaw;
float Yaw_Delta = 0;
float Pitch_0, Roll_0, Yaw_0;
float Pressure, Pressure_0 = 0;
int a=1;//K210复位引脚
extern struct MS5837_t MS5837;

extern osThreadId_t move_taskHandle;
extern osThreadId_t beep_taskHandle;
extern int Target_Yaw;
int16_t Time_Count = 0 ;
//int Base_Speed = 175;
extern int Base_Speed;
extern Kalman_Typedef Target_Flag_Filter;
void LED_Task(void *argument)
{
    /* USER CODE BEGIN */
    static float Yaw_Old = 0;

	osThreadSuspend(move_taskHandle);
	osDelay(20);
	Kalman_Init(&Target_Flag_Filter, 0.001, 0.001);

	Pitch_0 = Pitch;
	Roll_0 = Roll;
	Pressure_0 = Pressure;
	osThreadResume(move_taskHandle);
	osThreadResume(beep_taskHandle);
    /* Infinite loop */
    for (;;)
    {
        if(Time_Count < 30000)
        {
            Time_Count ++;
        }
        if(Time_Count == 8)
        {
            Yaw_0 = Yaw;
        }
		Yaw_Delta = Yaw - Yaw_Old;
		Yaw_Old = Yaw;
//		if(Time_Count == 25)
//		{
//			Base_Speed = 0;
//			WS_WriteAll_RGB(Range2Percentage(0, 255, 25), 0, 0);	//红
//			osDelay(1000);
//		}
//		WS_WriteAll_RGB(0, 0, 0);	
        osDelay(1000);
    }
    /* USER CODE END */	
}

extern osThreadId_t ukey_TaskHandle;
void UKEY_Task(void *argument)
{
    /* USER CODE BEGIN */
	//osThreadSuspend(ukey_TaskHandle);
    static unsigned char UKEY_Flag = 0;

    /* Infinite loop */
   for (;;)
    {
			//
		
//         if (HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin) == 0)
//        {
//            osDelay(20);
//            if (HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin) == 0)
//            {
//                UKEY_Flag = !UKEY_Flag;
//                {

//                }
//            }
//            while (HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin) == 0)
//            {
//                osDelay(1);
//            }
//            osDelay(20 - 1);
//        }
//        else
            osDelay(1);
    }
		
    /* USER CODE END */
}

short Temp = 0;
short aacx, aacy, aacz;
short gyrox, gyroy, gyroz;
short gyro[3] = {0};

void MPU6050_Task(void *argument)
{
    /* USER CODE BEGIN */
    /* Infinite loop */
    for (;;)
    {
		if(mpu_dmp_get_data(&Pitch, &Roll, &Yaw)==0)
   	{
			Pitch = -Pitch;
			Yaw = -Yaw;
			Temp = MPU_Get_Temperature();								//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);			//得到陀螺仪数据
			gyro[0] = gyrox;
			gyro[1] = -gyroy;
			gyro[2] = gyroz;
			
		}
		
        osDelay(5);
    }
    /* USER CODE END */
}

void OLED_Task(void *argument)
{
    /* USER CODE BEGIN */

    /* Infinite loop */
    for (;;)
    {
		oled_print_float(0, 6*8, Pitch);
		oled_print_float(2, 6*8, Roll);
		oled_print_float(4, 6*8, Yaw);
		oled_print_float(6, 6*8, Pressure);
		UpdateScreen(); 
        osDelay(17);
    }
    /* USER CODE END */
}

void BEEP_Task(void *argument)
{
    /* USER CODE BEGIN */
    osThreadSuspend(beep_taskHandle);
    /* Infinite loop */
    for (;;)
    {
			//k210复位
			
			 osDelay(10);
			/*
			
        HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, 0);
        osDelay(500);
        HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, 1);
        osThreadSuspend(beep_taskHandle);
			*/
    }
    /* USER CODE END */
}

extern osThreadId_t ms5837_read_tasHandle;
void MS5837_Read_Task(void *argument)
{
    /* USER CODE BEGIN */

    /* Infinite loop */
    for (;;)
    {
        MS5837_read(&MS5837_HI2C);
		Pressure = MS5837.pressure - Pressure_0;
        osDelay(5);
    }
    /* USER CODE END */
}

#define Start_Deep 4//3
#define End_Deep 22//20 //22
float Deep_Time = 100.0;
float Target_High = Start_Deep; 
//float Target_High = End_Deep;
//float Target_High = 10;

void Turn_Check_Task(void *argument)
{
    /* USER CODE BEGIN */
extern float Line_Angle_Filter;
extern int32_t Line_Angle;
extern int32_t Line_Key;
extern int32_t Line_x_Offs;
uint8_t Turn_Count = 0;
static uint8_t Turn_Times = 1;
    /* Infinite loop */
    for (;;)
    {
		if(Time_Count > 8 && fabs(Yaw_Delta) > 15)//转向以后，偏航角有角度差,一段时间以后
		{	
			while(Target_High < End_Deep - (End_Deep-Start_Deep)/(Deep_Time * 1000))
			{
				Target_High += (End_Deep-1)/(Deep_Time * 1000);
				if(Target_High > End_Deep)
					Target_High = End_Deep;
				osDelay(1);
			}
		}
        osDelay(1);
    }
    /* USER CODE END */
}

uint16_t Servo_PWM = 2500; //2500张开 ~ 4500收拢
extern TIM_HandleTypeDef htim2;
void Grap(uint8_t Flag)
{
	if (Flag)
		Servo_PWM = 5000;
	else
		Servo_PWM = 2500;

	//0---->180      1250---->6250
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 6250);

  }



#define RGB_Delay_Time 1000

#define RGB_Time_Delta 200
#define Light_Val 100

#define KLM_Flag 1
    	
void Blue_TRL(int num,int RGB_Time)//蓝色闪烁num次,间隔  RGB_Time_Delta   ms
{
	int i=0;
	for(i=0;i<num;i++)
	{
	    WS_WriteAll_RGB(0, 0,Range2Percentage(0, 255, Light_Val) );	//蓝色
			osDelay(RGB_Time);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time);
	}
		
}
void Green_ROL(int num,int RGB_Time)//绿色闪烁num次,间隔  RGB_Time_Delta   ms
{
	int i=0;
	for(i=0;i<num;i++)
	{
      WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val),0 );	//绿色，未验证
			osDelay(RGB_Time);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time);
	}
}
void Yello_ANN(int num,int RGB_Time)//黄色闪烁num次,间隔  RGB_Time_Delta   ms
{
	int i=0;
	for(i=0;i<num;i++)
	{
      WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val),0 );	//黄
			osDelay(RGB_Time);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time);
	}
}
void Red_REC(int num,int RGB_Time)//红色闪烁num次,间隔  RGB_Time_Delta   ms
{
	int i=0;
	for(i=0;i<num;i++)
	{
        WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time);
	}
}
///**************************************************************************
//函数功能：将角度转换为脉宽，进而改变PWM输出占空比
//入口参数：角度
//返回  值：无
//备注    ：角度范围为-90度到90度，当输入角度为-90度时，脉宽为0.5ms；当输入角度为0度时，脉宽为1.5ms；当输入角度为90度时，脉宽为2.5ms；
//**************************************************************************/
//double pulse_1;
//void translate_angle_to_pulse(double angle_1)
//{
//	pulse_1 = (((angle_1 + 90) / 90 ) + 0.5)*(50000/20);//参与姿态解算，控制旋转角度，范围（-90，90）20000

//}
///**************************************************************************
//函数功能：底层PWM输出
//入口参数：舵机目标角度
//返回  值：无
//备注    ：舵机控制频率为50Hz，周期为20ms，PSC = 72 - 1，ARR = 200 - 1，f = 72MHz /( PSC + 1 )( ARR +1 )
//				占空比 = pulse / ARR
//				当高电平时间为0.5ms时，舵机角度为0度
//				当高电平时间为1.5ms时，舵机角度为90度
//				当高电平时间为2.5ms时，舵机角度为180度
//**************************************************************************/
//void pwm_out(double angle_1)
//{
//	translate_angle_to_pulse(angle_1);
//	
//	if(pulse_1 != NULL)
//	{
//			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse_1);
//	}
//	
//}

     //WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			//WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val),0 );	//黄
			//	WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0,Range2Percentage(0, 255, Light_Val) );	//亮紫色
      //WS_WriteAll_RGB(0,0,Range2Percentage(0, 255, Light_Val) );	//青色
      //WS_WriteAll_RGB(0, 0,Range2Percentage(0, 255, Light_Val) );	//蓝色
//WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val),0 );	//绿色，未验证
#define Style     2

void WS2812_Task(void *argument)
{
    /* USER CODE BEGIN */
	extern int32_t Target_Flag, Line_Key;
	extern uint8_t RGB_Flag;
   int  flag=0;
    /* Infinite loop */
    for (;;)
    {	
		
			HAL_Delay(7000);
WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val));//看见白光
			HAL_Delay(650);
			WS_WriteAll_RGB(0,0,0);
			 pwm_out(0);//打捞收   
			if(Style==1&&flag!=1)
       {
				 flag=1;
			 HAL_Delay(5500);
			 Green_ROL(3,RGB_Time_Delta);//1圆
				 		 pwm_out(0);//打捞收
				  HAL_Delay(2500); 
				 //	pwm_out(-5);//打捞放
				 Blue_TRL(3,RGB_Time_Delta);//2三角形
			//    pwm_out(0);//打捞收
				  HAL_Delay(3000); 
				 //	pwm_out(-5);//打捞放
				 Green_ROL(3,RGB_Time_Delta);//3圆形
				// 	pwm_out(0);//打捞收
				  HAL_Delay(1000); 
				// 	pwm_out(-5);//打捞放
			   Red_REC(3,RGB_Time_Delta);//4.正方形
				//	pwm_out(0);//打捞收  
				 HAL_Delay(1000);
				// pwm_out(-5);//打捞放
		 	  Green_ROL(3,RGB_Time_Delta);//5.圆形
			//	 pwm_out(0);//打捞收 
			 HAL_Delay(500);
				// pwm_out(-5);//打捞放
			 Red_REC(3,RGB_Time_Delta);//6.正方形
				// 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
				//	 pwm_out(-4);//打捞放
				 Red_REC(3,RGB_Time_Delta);//7.圆形.
				 	
			//	 pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-80);//打捞放
				 
				 Blue_TRL(3,RGB_Time_Delta);//8三角形
			   
	  	//	pwm_out(0);//打捞收					
				HAL_Delay(1500); 
				pwm_out(-80);//打捞放
					
				Yello_ANN(3,RGB_Time_Delta);//9圆环 
				
				 	// 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
				 	pwm_out(-80);//打捞放
					
				 Red_REC(3,RGB_Time_Delta);//10.正方形
				 pwm_out(0);//打捞收
			 }
			 else if(Style==2&&flag!=2)
			 {
				 flag=2;
				HAL_Delay(5500);
			  Red_REC(3,RGB_Time_Delta);//1正方形
				 		 pwm_out(0);//打捞收
				  HAL_Delay(2500); 
				
				 Blue_TRL(3,RGB_Time_Delta);//2三角形
			  //  pwm_out(0);//打捞收
				  HAL_Delay(3000); 
	
				 Green_ROL(3,RGB_Time_Delta);//3圆形
				// 	pwm_out(0);//打捞收
				  HAL_Delay(1000); 
			
			   Blue_TRL(3,RGB_Time_Delta);//4.三角形
					pwm_out(0);//打捞收  
				 HAL_Delay(1000);
			//	 pwm_out(-10);//打捞放
		 	  Green_ROL(3,RGB_Time_Delta);//5.圆形
				 pwm_out(0);//打捞收 
			 HAL_Delay(500);
				// pwm_out(-10);//打捞放
			 Yello_ANN(3,RGB_Time_Delta);//6.圆环
			//	 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
			 	pwm_out(-80);//打捞放
				 Blue_TRL(3,RGB_Time_Delta);//7.三角形.
				 	
			//	 pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-80);//打捞放
				 
				 Blue_TRL(3,RGB_Time_Delta);//8三角形
			 
	  	//	pwm_out(0);//打捞收					
				HAL_Delay(1500); 
				pwm_out(-90);//打捞放
					
				Yello_ANN(3,RGB_Time_Delta);//9圆环 
				
				 //	 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
				 	pwm_out(-80);//打捞放
					
				 Red_REC(3,RGB_Time_Delta);//10.正方形
			 }
			 else if(Style==3&&flag!=3)
			 {
				 flag=3;
				HAL_Delay(5500);
			  Green_ROL(3,RGB_Time_Delta);//1圆
				 		 pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-10);//打捞放
				 Blue_TRL(3,RGB_Time_Delta);//2三角形
			    pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-10);//打捞放
				 Yello_ANN(3,RGB_Time_Delta);//3圆环
				 	pwm_out(0);//打捞收
				  HAL_Delay(1000); 
				 	pwm_out(-10);//打捞放
			   Red_REC(3,RGB_Time_Delta);//4.正方形
					pwm_out(0);//打捞收  
				 HAL_Delay(1000);
				 pwm_out(-10);//打捞放
		 	  Green_ROL(3,RGB_Time_Delta);//5.圆形
				 pwm_out(0);//打捞收 
			 HAL_Delay(500);
				 pwm_out(-10);//打捞放
			 Yello_ANN(3,RGB_Time_Delta);//6.圆环
				 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
				 	pwm_out(-80);//打捞放
				 Red_REC(3,RGB_Time_Delta);//7.正方形.
				 	
				 pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-80);//打捞放
				 
				 Green_ROL(3,RGB_Time_Delta);//8圆形
			 
	  		pwm_out(0);//打捞收					
				HAL_Delay(1500); 
				pwm_out(-80);//打捞放
					
				Yello_ANN(3,RGB_Time_Delta);//9圆环 
				
				 	 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
				 	pwm_out(-80);//打捞放
					
				 Blue_TRL(3,RGB_Time_Delta);//10.三角形
			 }
			  else if(Style==4&&flag!=4)
			 {
				 flag=4;
				HAL_Delay(5500);
			  Blue_TRL(3,RGB_Time_Delta);//1三角形
				 		 pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-10);//打捞放
				 Blue_TRL(3,RGB_Time_Delta);//2三角形
			    pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-10);//打捞放
				 Yello_ANN(3,RGB_Time_Delta);//3圆环
				 	pwm_out(0);//打捞收
				  HAL_Delay(1000); 
				 	pwm_out(-10);//打捞放
			   Blue_TRL(3,RGB_Time_Delta);//4.三角形
					pwm_out(0);//打捞收  
				 HAL_Delay(1000);
				 pwm_out(-10);//打捞放
		 	  Yello_ANN(3,RGB_Time_Delta);//5.圆环
				 pwm_out(0);//打捞收 
			 HAL_Delay(500);
				 pwm_out(-10);//打捞放
			 Red_REC(3,RGB_Time_Delta);//6.正方形
				 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
				 	pwm_out(-80);//打捞放
				 Yello_ANN(3,RGB_Time_Delta);//7.圆环.
				 	
				 pwm_out(0);//打捞收
				  HAL_Delay(2000); 
				 	pwm_out(-80);//打捞放
				    
				 Green_ROL(3,RGB_Time_Delta);//8圆形  
			 
	  		pwm_out(0);//打捞收					
				HAL_Delay(1500); 
				pwm_out(-80);//打捞放
					
				Yello_ANN(3,RGB_Time_Delta);//9圆环 
				
				 	 	pwm_out(0);//打捞收
				HAL_Delay(1500); 
				 	pwm_out(-80);//打捞放
					
				 Blue_TRL(3,RGB_Time_Delta);//10.三角形
			 }
			
			
		//	pwm_out(180);
	
			
			
#if KLM_Flag	
			#else		
//		if(Target_Flag_Filter.out <= 0.5 && RGB_Flag != 1)	//四边形
	double ab=Target_Flag_Filter.out ;
				if(Target_Flag_Filter.out <= 3.5 && RGB_Flag != 1)	//四边形
			{
			RGB_Flag = 1;
			osDelay(RGB_Delay_Time);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
		}
		//else if(1.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 2.5 && RGB_Flag != 2)	//圆环
		else if(4.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 5.5 && RGB_Flag != 2)	//圆环
		{
			RGB_Flag = 2;
			osDelay(RGB_Delay_Time);
			Grap(0);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
		}	
		//else if(2.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 3.5 && RGB_Flag != 3)	//三角形
		else if(5.5<= Target_Flag_Filter.out && Target_Flag_Filter.out <= 7 && RGB_Flag != 3)	//三角形
		{
			RGB_Flag = 3;
			osDelay(RGB_Delay_Time);
			Grap(0);//控制舵机
			WS_WriteAll_RGB(0, 0,Range2Percentage(0, 255, Light_Val) );	//蓝色
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0,Range2Percentage(0, 255, Light_Val) );	//蓝色
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0,Range2Percentage(0, 255, Light_Val) );	//蓝色
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0,Range2Percentage(0, 255, Light_Val) );	//蓝色
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0,Range2Percentage(0, 255, Light_Val) );	//蓝色
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
		}	
		//else if(3.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 4.5 && RGB_Flag != 4)	//三角形
		else if(Target_Flag_Filter.out >= 9  && RGB_Flag != 4)	//三角形
		{
			RGB_Flag = 4;
			osDelay(RGB_Delay_Time);
			Grap(0);//控制舵机
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val),0 );	//黄
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val),0 );	//黄
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val),0 );	//黄
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val),0 );	//黄
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), Range2Percentage(0, 255, Light_Val),0 );	//黄
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
		}
		else
		{
			WS_WriteAll_RGB(0, 0, 0);
			WS_WriteAll_RGB(0, 0, 0);
			RGB_Flag = 0;
			osDelay(1000);
      }
#endif
#if KLM_Flag	
#else
	//if(Target_Flag_Filter.out <= 3.5 && RGB_Flag != 1)	//三棱柱
	if(Target_Flag_Filter.out <= 0.5 && RGB_Flag != 1)	//三棱柱
		{
			RGB_Flag = 1;
			osDelay(RGB_Delay_Time);
				pwm_out(-10);
      Red(5,RGB_Time_Delta);
				pwm_out(-90);
			Target_Flag_Filter.out=1;//归1
		}
		else if(1.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 2.5 && RGB_Flag != 2)	//圆柱
		//else if(4.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 5.5 && RGB_Flag != 2)	//圆柱
		{
			RGB_Flag = 2;
			osDelay(RGB_Delay_Time);
			pwm_out(-10);
			Green(5,RGB_Time_Delta);
		pwm_out(-90);
			Target_Flag_Filter.out=1;//归1
		}	
		else if(2.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 3.5 && RGB_Flag != 3)	//圆环
		//else if(5.5<= Target_Flag_Filter.out && Target_Flag_Filter.out <= 7 && RGB_Flag != 3)	//圆环
		{
			RGB_Flag = 3;
			osDelay(RGB_Delay_Time);
			pwm_out(-10);//控制舵机
      Blue(5,RGB_Time_Delta);//蓝色闪烁5次间隔，，ms
			pwm_out(-90);
		  Target_Flag_Filter.out=1;//归1
			
		}	
		else if(3.5 <= Target_Flag_Filter.out && Target_Flag_Filter.out <= 4.5 && RGB_Flag != 4)	//正方形
	//	else if(Target_Flag_Filter.out >= 9  && RGB_Flag != 4)	//形
		{
			RGB_Flag = 4;
			osDelay(RGB_Delay_Time);
			pwm_out(-10);
	  Yello(5,RGB_Time_Delta);
			pwm_out(-90);
		Target_Flag_Filter.out=1;//归1
			
		}
		else//什么都没有识别
		{
			osDelay(RGB_Delay_Time);
			WS_WriteAll_RGB(0, 0, 0);
			WS_WriteAll_RGB(0, 0, 0);
			RGB_Flag = 0;
			osDelay(1000);
			pwm_out(-90);//复位
			Target_Flag_Filter.out=1;//归1
		}

#endif
		
	
		
#if KLM_Flag	
		#else
		if(Target_Flag == 0 && RGB_Flag != 1)	//圆
		{
			RGB_Flag = 1;
			osDelay(RGB_Delay_Time);
			Grap(0);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
		}
		else if(Target_Flag == 2 && RGB_Flag != 2)	//方
		{
			RGB_Flag = 2;
			osDelay(RGB_Delay_Time);
			Grap(0);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, Range2Percentage(0, 255, Light_Val), 0);	//绿
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
		}
		else if(Target_Flag == 1 && RGB_Flag != 0 && Line_Key == 0)	//啥也看不到
		{
//			WS_WriteAll_RGB(0, 0, 0);
			WS_WriteAll_RGB(0, 0, 0);
			RGB_Flag = 0;
			Grap(0);
			osDelay(1000);
		}
		else if(Target_Flag == 1 && RGB_Flag != 3 && Line_Key == 1)	//只看到管子
		{
//			WS_WriteAll_RGB(0, 0, Range2Percentage(0, 255, 25));
			WS_WriteAll_RGB(0, 0, 0);
			RGB_Flag = 3;
			Grap(0);
			osDelay(1000);
		}	
   #endif
		
        osDelay(1);
    }
    /* USER CODE END */
}
