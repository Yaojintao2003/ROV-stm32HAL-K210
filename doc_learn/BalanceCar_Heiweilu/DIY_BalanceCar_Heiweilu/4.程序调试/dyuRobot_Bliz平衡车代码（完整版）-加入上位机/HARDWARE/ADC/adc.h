#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#define Battery_Ch 8		//通道8
void Adc_Init(void);
u16 Get_Adc(u8 ch);
float Get_battery_volt(void);  
#endif 
