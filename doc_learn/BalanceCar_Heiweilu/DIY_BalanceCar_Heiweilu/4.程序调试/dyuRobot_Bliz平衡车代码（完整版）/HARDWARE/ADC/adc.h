#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
#define Battery_Ch 8		//ͨ��8
void Adc_Init(void);
u16 Get_Adc(u8 ch);
float Get_battery_volt(void);  
#endif 
