/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



//-----ȫ�ֱ���---
__IO uint32_t uwTick_UART = 0;
int Vertical_out,Velocity_out,Turn_out; // ֱ����&�ٶȻ�&ת�򻷵��������
int Encoder_Left,Encoder_Right,Balance_PWM;
int Moto1,Moto2;
int PWM_out,Turn_Pwm; //�ջ����
		
u8 AIN1,AIN2;//���Ƶ������ת
u8 BIN1,BIN2;
u8 Moto_Flag=0;//������Ʊ�־
u8 Start_Flag=0;//ϵͳ��ʼ����ɱ�־

float Pitch,Roll,Yaw;	        // Pitch�������ǣ�Roll������ǣ�Yaw��ƫ����
short gyrox,gyroy,gyroz;        // ���ٶ�
short aacx,aacy,aacz;           // ���ٶ�
float Med_Angle=0;//��е��ֵ����ʹ��С������ƽ��ס�ĽǶ� 

uint8_t Usart1_RX_Buf;  //���ڽ������ݻ���buf
u8 Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;//����ң�ر�־
unsigned char Usart_TX_Buf[40];


//----�Ӻ�������-----
void UART_Proc();
//void	go();
//void	stop();
//void	backward();
int Vertical(float Angle,float Gyro_Y);			 				//ֱ����
int Velocity(int Encoder_left,int Encoder_right);				//�ٶȻ�
int Turn(int Encoder_Left,int Encoder_Right,float gyro);	//ת��
u8 Stop(signed int angle);    		//���±���
void Limit(int *motoA,int *motoB);  //����ٶ��޷�
void Set_Pwm(int Moto1,int Moto2);	//����PWM�������
	



//-----printf�ض���-----
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);
	return ch;

}






/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //����TIM��ʼ��
    HAL_TIM_Base_Start_IT(&htim2);//ÿ10ms����һ���ж� 
  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//PWM
  	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	
  
	 HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);//������
	 HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	 HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
	 HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);	
//  
//    HAL_UART_Receive_IT(&huart1, &Usart1_RX_Buf, 1);//ʹ�ܴ���1�����ж�
	 
//	 sprintf((char *)Usart_TX_Buf,"AT+UART=115200,0,0\r\n");
//    HAL_UART_Transmit(&huart1,(unsigned char*)Usart_TX_Buf,strlen(str),8);	 
	 
	 

	//------MPU6050����-----
//   printf("Mpu6050 Project Start\r\n");

  //----iic��ȡMUP����ID----
  uint8_t recv = 0x00;
  HAL_I2C_Mem_Read(&hi2c2, (0x68 << 1), 0x75, I2C_MEMADD_SIZE_8BIT, &recv, 1, 0xfff);//�����MUP��ַ,����ID�Ĵ���
  
  if (recv == 0x68)
  {
//    printf("mpu6050 ID Read: OK at 0x68\r\n");	  
  }
  else
  {
//    printf("Err mpu id:0x%x\r\n", recv);  
  }  	 
  
  //-----DMP��ʼ��----
  while(mpu_dmp_init())//�ɹ�����0�����򷵻�1
  {
		uint8_t res;
		res = mpu_dmp_init();
		HAL_Delay(300);
//		printf("res=%d\r\n",res);
//	   printf("-----------------------3333------------\r\n");
		  
  }
// printf("---------------MPU��ʼ���ɹ�����1---------------\r\n");
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0); //��PC13 LED
 Start_Flag = 1; //��־ϵͳ��ʼ���ɹ�
 


  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	UART_Proc();
  

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/**************************************************************************
�������ܣ�10ms��ʱ���жϣ������ƺ���
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if(htim->Instance == TIM2)//TIM2����Ϊ10ms�ж�
  {
		// 1.�ɼ�����������&MPU6050�Ƕ���Ϣ
   // �������԰�װ���պ����180�ȣ�Ϊ�˱������������һ�£�����Ҫ������һ��ȡ��  	

	  mpu_dmp_get_data(&Pitch,&Roll,&Yaw);	    // ��ȡ�Ƕ�
     MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  // ��ȡ���ٶ�
     MPU_Get_Accelerometer(&aacx,&aacy,&aacz); // ��ȡ���ٶ�
	  
	  	Encoder_Left  =   Read_Encoder(3);		//��ȡ�����������ĵ��ת��
		Encoder_Right =   Read_Encoder(4);		

  // 2.������ѹ��ջ������У���������������
//	  Velocity_out = Velocity(Encoder_Left,Encoder_Right);// �ٶȻ�������  
	  Vertical_out = Vertical(Roll,gyrox+62);// ֱ�������PWM
		
	  // -------------��������--------------
//	  if(1==Flag_left||1==Flag_right)    
//			Turn_Pwm =Turn(Encoder_Left,Encoder_Right,gyroz+33);//��������ת��־�Ž���ת�򻷱ջ�����
//	  else   
//			Turn_Pwm=-0.5*(gyroz+33); //������ֱ��
	  
	  
		//------------�������----------------
	  PWM_out= Vertical_out+Velocity_out;

	  // 3.�ѿ�����������ص�����ϣ�������տ���
		Moto1 = PWM_out+Turn_Pwm; // ����
      Moto2 = PWM_out-Turn_Pwm; // �ҵ��  
      Limit(&Moto1,&Moto1);     // PWM�޷� 
		Set_Pwm(Moto1,Moto2);        // ���ص������ 
  }



}

/*************************************************************************** 
ֱ����PD��������Kp*Ek+Kd*Ek_D
��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
���ڣ�PMW��ֵ
******************************************************************************/
int Vertical(float Angle,float Gyro_Y) 
{
	float Vertical_Kp= -550,Vertical_Kd= -02;	//ֱ����Kp,Kd
	PWM_out = Vertical_Kp*(Angle-0.25)+Vertical_Kd*(Gyro_Y-0);	
	return PWM_out;	
} 


/********************************************************************* 
�ٶȻ�PI��������Kp*Ek+Ki*Ek_S(Ek_S��ƫ��Ļ���)
��ڣ����ұ������⵽����ֵ
���ڣ�                   
**********************************************************************/
int Velocity(int Encoder_left,int Encoder_right)	
{
	// ����ɾ�̬�����������ھ�̬�洢����ʹ�ñ���������
	static float Velocity,Encoder_Err,Encoder,Encoder_last,Movement=0; //�ٶȣ���������
	static float Encoder_Integral;					  //��������ֵ����
		
	
	float kp=10,ki=0.05;	
	
	//--------------��������----------------
	if(1==Flag_forward) 		    Movement = -200;
	else if(1==Flag_retreat)    Movement = 200;
	else							    Movement = 0;
	
	
	// 1.�����ٶ�ƫ�� 	
	Encoder_Err = ((Encoder_Left+Encoder_Right)-0);	
 
	// 2.���ٶ�ƫ�����--��ͨ�˲�--
  // low_out = (1-a)*Ek+a*low_out_last	
	Encoder = Encoder_Err*0.3 + Encoder_last*0.7;// ʹ�ò��θ���ƽ��
	Encoder_last = Encoder; 							// ��ֹ�ٶȹ���Ӱ��ֱ��������������

	// 3.���ٶ�ƫ����ֳ�λ��,ң�ص��ٶ�ͨ�����������ٶȿ������������ٶ�ͻ���ֱ�����Ƶ�Ӱ��	
  Encoder_Integral += Encoder-Movement;	


	
	

	// 4.�����޷�	
	if(Encoder_Integral>3000)  	Encoder_Integral=3000;   
	if(Encoder_Integral<-3000)	   Encoder_Integral=-3000;           	

	if(Moto_Flag == 1||Start_Flag ==0) 			Encoder_Integral=0;     		//===����رպ���߸�λ�������
    //5.�ٶȻ��������	
  Velocity=Encoder*kp+Encoder_Integral*ki;

	
	
	return Velocity;



}


/*********************************************************************
ת�򻷣�ϵ��*Z����ٶ�+ϵ��*ң������
��ڣ����ҵ�������������ֵ��Z����ٶ�
**********************************************************************/
int Turn(int Encoder_Left,int Encoder_Right,float gyro)
{
   float Turn_Target,Turn_PWM,Bias,Encoder_temp,Turn_Convert=70,Turn_Count; 
   float Turn_Amplitude=100,Turn_Kp=10,Turn_Kd=0;  
	
	// --- �������� ---
	if(1==Flag_left) 			Turn_Target+=Turn_Convert;//��ת��־
	else if(1==Flag_right)  Turn_Target-=Turn_Convert; //��ת��־
	 else 						 Turn_Target=0;
	
	 if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude; //ת���޷�    
    if(Turn_Target<-Turn_Amplitude)  Turn_Target=-Turn_Amplitude;

	 if(Flag_forward==1||Flag_retreat==1) Turn_Kd=7;		          
    else 										  Turn_Kd=0;//ǰ�����ߺ��˱�־
	


	
	 //=============turing PD controller==================//	
		Turn_PWM= -Turn_Target*Turn_Kp+gyro*Turn_Kd;
 
  return Turn_PWM;
  
  
  

}



	
	
/*************************************************************************** 
�������ܣ����Ƶ��
******************************************************************************/
void Contrl(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, AIN1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, AIN2);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, BIN1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, BIN2);	
		

}




/**************************************************************************
�������ܣ�����쳣�رպ���
��ڲ������Ƕ�
����  ֵ��1���رգ�0�����ر�
**************************************************************************/	 		
u8 Stop(signed int angle)
{
	    u8 temp=0;
			if(angle<-50||angle>50)
			{	                                //===��Ǵ���40�ȹرյ��
				temp=1;                   		   //===Flag_Stop��1�رյ��
				Moto1 = 0;
				Moto2 = 0;
      }
	
		return temp;
}



/**************************************************************************
�������ܣ����ת�����ƺ���
��ڲ������ջ������������ֵ
**************************************************************************/	
void Set_Pwm(int Moto1,int Moto2)
{
	
	 int dead_zone = 3000 ;		//L298N�������� 0 - 3000
	 Moto_Flag=Stop(Roll);	//��ȡ�Ƿ��µı�־
	if(Start_Flag == 1)		//һ���ж�ϵͳ�Ƿ�������ʼ��
	{
		if(Moto_Flag==0)	//�����ж�
		{
			if(Moto1>0)  AIN1 = 1,AIN2 = 0;		
			else			 AIN1 = 0,AIN2 = 1;							
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,abs(Moto1)+dead_zone);		 
			
			if(Moto2>0)  BIN1 = 1,BIN2 = 0;		
			else		    BIN1 = 0 ,BIN2 = 1;
    		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,abs(Moto1)+dead_zone); //0-7200
		}
			
		else//���¾͹رյ��
		{
		   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);	//4500-6000
		   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);	//4500-6000
		}
		 Contrl();
	}
}


/**************************************************************************
�������ܣ����Ƶ���ٶ�
��ڲ������ջ������������ֵ
**************************************************************************/	
void Limit(int *motoA,int *motoB)
{
	if(*motoA>7000)*motoA=7000;//���7200
	if(*motoA<-7000)*motoA=-7000;

	if(*motoB>7000)*motoB=7000;
	if(*motoB<-7000)*motoB=-7000;
}

/**************************************************************************
�������ܣ�����ң������USART1�����жϻص�
���ţ�		 TX_PA9,RX_PA10
**************************************************************************/
//�����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	switch(Usart1_RX_Buf)
    {
        case 0:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
        break;
        case 1:
        Flag_forward = 1, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        break;
        case 2:
        Flag_forward = 0, Flag_retreat = 1, Flag_left = 0, Flag_right = 0;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        break;
        case 3:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 1, Flag_right = 0;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        break;
        case 4:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 1;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        break;
        default:
        Flag_forward = 0, Flag_retreat = 0, Flag_left = 0, Flag_right = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,1);
        break;
    }
    HAL_UART_Receive_IT(&huart1, &Usart1_RX_Buf, 1);
}













/**************************************************************************
�������ܣ�ͨ�����ڲ�������
**************************************************************************/
void UART_Proc()
{
  if((uwTick - uwTick_UART) < 15) return;
	uwTick_UART = uwTick;
	
		printf("%d\r\n",PWM_out);
//		printf("1111\n");	
	
//	printf("-------------------------------------------\r\n");	
//   if ((mpu_dmp_get_data(&Pitch, &Roll, &Yaw) == 0) && (MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz) == 0)) //DMP��ȡŷ���ǣ�digital motion processor�����˶�������
//    {
//		//�����ǣ�����ǣ�ƫ����
////		 printf("pitch=%.3f   roll=%.3f   yaw=%.3f \r\n", Pitch, Roll, Yaw);   //���ڴ�ӡŷ����
////   	 printf("gyrox=%d   gyroy=%d  gyroz=%d \r\n", gyrox, gyroy, gyroz);	// ���ٶ�
////		 printf("-------------------------------------------\r\n");
//			
//	 }
//	  printf("Vertical_out=%d",Vertical_out);

}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

