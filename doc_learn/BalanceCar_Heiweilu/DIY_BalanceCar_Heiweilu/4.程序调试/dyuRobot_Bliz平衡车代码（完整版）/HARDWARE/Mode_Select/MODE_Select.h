#ifndef __MODE_SELECT_H
#define __MODE_SELECT_H
#include "sys.h"
/**************************************************************************
作    者：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
extern unsigned char BMP_0[];
extern unsigned char BMP_20[];
extern unsigned char BMP_40[];
extern unsigned char BMP_60[];
extern unsigned char BMP_80[];
extern unsigned char BMP_100[];
extern unsigned char BMP_hello[];
extern unsigned char BMP_smile[];
extern unsigned char BMP_cry_Ultrasonic[];
extern unsigned char BMP_cry_fall[];
extern unsigned char BMP_bm[];


extern u8 flag_UltrasonicWave;//超声波是否超出安全距离标志位
extern u8 flag_fall;					 //摔倒标志位
extern u8 key;								 //按键的键值
extern u8 KEY_MODE;					 //模式0,HELLO BLIZ
													//模式1，电量
												 //模式2，表情模式
												 //模式3，参数模式
extern u8 DIS_STATE;		 //用来实现只刷一次屏幕的变量			

void KEY_mode_Select(void);
#endif
