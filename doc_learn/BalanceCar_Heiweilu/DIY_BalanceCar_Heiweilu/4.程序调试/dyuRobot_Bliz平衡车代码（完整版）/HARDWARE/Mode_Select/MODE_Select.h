#ifndef __MODE_SELECT_H
#define __MODE_SELECT_H
#include "sys.h"
/**************************************************************************
��    �ߣ��������
�Ա���ַ��https://shop119207236.taobao.com
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


extern u8 flag_UltrasonicWave;//�������Ƿ񳬳���ȫ�����־λ
extern u8 flag_fall;					 //ˤ����־λ
extern u8 key;								 //�����ļ�ֵ
extern u8 KEY_MODE;					 //ģʽ0,HELLO BLIZ
													//ģʽ1������
												 //ģʽ2������ģʽ
												 //ģʽ3������ģʽ
extern u8 DIS_STATE;		 //����ʵ��ֻˢһ����Ļ�ı���			

void KEY_mode_Select(void);
#endif
