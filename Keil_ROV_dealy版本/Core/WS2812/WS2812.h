#ifndef _WS2812_H
#define _WS2812_H

#include "main.h"

/*设置定时器周期为1.25us*/
#define WS2812_Counter_Period 125
#define WS2812_Htim htim4
#define WS2812_CH	TIM_CHANNEL_1
#define PIXEL_NUM  18  //控制灯珠的数目

#define GRB  24						//3*8
#define NUM (24*PIXEL_NUM + 250)	// Reset >300us 300us / 1.25us = 240
/*
 * TH+TL=1.1us±300ns
 * T0H	0码，高电平时间220ns-420ns
 * T1H	1码，高电平时间750ns-1.6ns
 * T0L	0码，低电平时间750ns-1.6ns
 * T1L	1码，低电平时间220ns-420ns
 * */
#define WS1  (WS2812_Counter_Period/3*2-1)
#define WS0  (WS2812_Counter_Period/3-1)

extern uint16_t send_Buf[NUM];

void WS_Load(void);
void WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B);// 全部led灯设置成一样的亮度，其中RGB分别设置亮度 WS2812的写入顺序是GRB，高位在前面
void WS_CloseAll(void);

uint32_t WS281x_Color(uint8_t red, uint8_t green, uint8_t blue);
void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor);////混合色直接输入
void WS281x_SetPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue);////设置单个的rgb

uint32_t Wheel(uint8_t WheelPos);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);

#endif

