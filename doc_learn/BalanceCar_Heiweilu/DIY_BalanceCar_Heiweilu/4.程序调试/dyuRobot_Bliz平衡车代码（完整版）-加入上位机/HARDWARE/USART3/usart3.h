#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	
 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
extern u8 Usart3_Receive;
void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void Uart3SendByte(char byte);   //���ڷ���һ���ֽ�
void Uart3SendBuf(char *buf, u16 len);
void Uart3SendStr(char *str);
#endif

