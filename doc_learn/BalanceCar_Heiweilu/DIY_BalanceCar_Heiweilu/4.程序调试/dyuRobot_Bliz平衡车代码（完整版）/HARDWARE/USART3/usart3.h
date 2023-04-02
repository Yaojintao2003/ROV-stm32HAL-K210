#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
extern u8 Usart3_Receive;
void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void Uart3SendByte(char byte);   //串口发送一个字节
void Uart3SendBuf(char *buf, u16 len);
void Uart3SendStr(char *str);
#endif

