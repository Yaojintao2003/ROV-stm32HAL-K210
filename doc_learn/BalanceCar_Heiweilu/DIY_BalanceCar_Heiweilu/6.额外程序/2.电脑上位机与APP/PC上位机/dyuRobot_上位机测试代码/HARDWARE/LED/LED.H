#ifndef __LED_H
#define __LED_H
#include "sys.h"
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
//LED 端口定义
#define LED PCout(14) // PC14
void LED_Init(void);  //初始化
void Led_Flash(u16 time);
#endif
