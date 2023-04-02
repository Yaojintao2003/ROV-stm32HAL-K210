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
    HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, 1);
	HAL_GPIO_WritePin(K210_RST_GPIO_Port, K210_RST_Pin, 1);
	osDelay(20);
	HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, 0);
	HAL_GPIO_WritePin(K210_RST_GPIO_Port, K210_RST_Pin, 0);
	Kalman_Init(&Target_Flag_Filter, 0.001, 0.001);
	ESC_Init(ESC_RTOS);

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
	osThreadSuspend(ukey_TaskHandle);
    static unsigned char UKEY_Flag = 0;

    /* Infinite loop */
    for (;;)
    {
        if (HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin) == 0)
        {
            osDelay(20);
            if (HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin) == 0)
            {
                UKEY_Flag = !UKEY_Flag;
                {

                }
            }
            while (HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin) == 0)
            {
                osDelay(1);
            }
            osDelay(20 - 1);
        }
        else
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
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
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
    osThreadSuspend(beep_taskHandle);//阻塞
    /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, 0);
        osDelay(500);
        HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, 1);
        osThreadSuspend(beep_taskHandle);
    }
    /* USER CODE END */
}

extern osThreadId_t ms5837_read_tasHandle;//深度传感器
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

#define Start_Deep 3
#define End_Deep 20 //22
float Deep_Time = 10.0;
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
		if(Time_Count > 8 && fabs(Yaw_Delta) > 15)
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
	//Left  4500 ~ 2500 (2500 ~ 7500)
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 7000 - Servo_PWM);	
	//Right 2500 ~ 4500 (2500 ~ 7500)
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Servo_PWM);	
}

#define RGB_Delay_Time 1000

#define RGB_Time_Delta 200
#define Light_Val 100

#define KLM_Flag 1

void WS2812_Task(void *argument)
{
    /* USER CODE BEGIN */
	
	 for (;;)
    {
		WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
		//	WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
		//	WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
		//	WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
		//	WS_WriteAll_RGB(0, 0, 0);
			osDelay(RGB_Time_Delta);
		//	WS_WriteAll_RGB(Range2Percentage(0, 255, Light_Val), 0, 0);	//红
			osDelay(RGB_Time_Delta);
			WS_WriteAll_RGB(0, 0, 0);
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
        osDelay(1);
    }
    /* USER CODE END */
}
