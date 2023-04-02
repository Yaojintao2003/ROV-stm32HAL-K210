#ifndef _WS2812_H
#define _WS2812_H

#include "main.h"

/*���ö�ʱ������Ϊ1.25us*/
#define WS2812_Counter_Period 125
#define WS2812_Htim htim4
#define WS2812_CH	TIM_CHANNEL_1
#define PIXEL_NUM  18  //���Ƶ������Ŀ

#define GRB  24						//3*8
#define NUM (24*PIXEL_NUM + 250)	// Reset >300us 300us / 1.25us = 240
/*
 * TH+TL=1.1us��300ns
 * T0H	0�룬�ߵ�ƽʱ��220ns-420ns
 * T1H	1�룬�ߵ�ƽʱ��750ns-1.6ns
 * T0L	0�룬�͵�ƽʱ��750ns-1.6ns
 * T1L	1�룬�͵�ƽʱ��220ns-420ns
 * */
#define WS1  (WS2812_Counter_Period/3*2-1)
#define WS0  (WS2812_Counter_Period/3-1)

extern uint16_t send_Buf[NUM];

void WS_Load(void);
void WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B);// ȫ��led�����ó�һ�������ȣ�����RGB�ֱ��������� WS2812��д��˳����GRB����λ��ǰ��
void WS_CloseAll(void);

uint32_t WS281x_Color(uint8_t red, uint8_t green, uint8_t blue);
void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor);////���ɫֱ������
void WS281x_SetPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue);////���õ�����rgb

uint32_t Wheel(uint8_t WheelPos);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);

#endif

