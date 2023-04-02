/*
 * ESC.h
 *
 *  Created on: Jan 16, 2021
 *      Author: Royic
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

typedef enum
{
    Motor_AW = 1,
    Motor_DW,
    Motor_AS,
    Motor_DS,
    Motor_LE,
    Motor_RI,
} Motor_Type;

typedef enum
{
    ESC_RTOS = 0,
    ESC_main,
} ESC_Type;  

void ESC_Init(ESC_Type Flag);
void translate_angle_to_pulse(double angle_1);
void pwm_out(double angle_1);
#endif /* INC_ESC_H_ */
