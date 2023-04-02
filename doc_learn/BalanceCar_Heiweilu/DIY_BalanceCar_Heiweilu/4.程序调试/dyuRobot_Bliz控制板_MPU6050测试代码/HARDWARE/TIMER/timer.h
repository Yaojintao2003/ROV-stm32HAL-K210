#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/

void TIM3_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
 
#endif
