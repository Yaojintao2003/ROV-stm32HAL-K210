/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Board_Pin GPIO_PIN_13
#define LED_Board_GPIO_Port GPIOC
#define UKEY_Pin GPIO_PIN_0
#define UKEY_GPIO_Port GPIOA
#define Servo_1_Pin GPIO_PIN_1
#define Servo_1_GPIO_Port GPIOA
#define Side_Motor_1_Pin GPIO_PIN_2
#define Side_Motor_1_GPIO_Port GPIOA
#define Side_Motor_2_Pin GPIO_PIN_3
#define Side_Motor_2_GPIO_Port GPIOA
#define Servo_2_Pin GPIO_PIN_5
#define Servo_2_GPIO_Port GPIOA
#define Main_Motor_1_Pin GPIO_PIN_6
#define Main_Motor_1_GPIO_Port GPIOA
#define Main_Motor_2_Pin GPIO_PIN_7
#define Main_Motor_2_GPIO_Port GPIOA
#define Main_Motor_3_Pin GPIO_PIN_0
#define Main_Motor_3_GPIO_Port GPIOB
#define Main_Motor_4_Pin GPIO_PIN_1
#define Main_Motor_4_GPIO_Port GPIOB
#define SCR12864_SCL_Pin GPIO_PIN_10
#define SCR12864_SCL_GPIO_Port GPIOB
#define B30_SCL_Pin GPIO_PIN_8
#define B30_SCL_GPIO_Port GPIOA
#define K210_RST_Pin GPIO_PIN_15
#define K210_RST_GPIO_Port GPIOA
#define SCR12864_SDA_Pin GPIO_PIN_3
#define SCR12864_SDA_GPIO_Port GPIOB
#define B30_SDA_Pin GPIO_PIN_4
#define B30_SDA_GPIO_Port GPIOB
#define WS2812_DI_Pin GPIO_PIN_6
#define WS2812_DI_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_7
#define MPU6050_SDA_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_8
#define MPU6050_SCL_GPIO_Port GPIOB
#define Beep_Pin GPIO_PIN_9
#define Beep_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
