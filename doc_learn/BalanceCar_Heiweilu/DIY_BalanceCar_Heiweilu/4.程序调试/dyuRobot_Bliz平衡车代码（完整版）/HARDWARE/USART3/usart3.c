#include "usart3.h"
 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��UGPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3ʱ��
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);

   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //���յ�����
	{	  
		 static	int uart_receive=0;//����������ر���
		 uart_receive=USART_ReceiveData(USART3); 
		if(uart_receive==0x00)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
		else if(uart_receive==0x01)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
		else if(uart_receive==0x05)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
		else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
													Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
		else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //��
													Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
		else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;		//////////////ɲ��

	}  											 
} 

void Uart3SendByte(char byte)   //���ڷ���һ���ֽ�
{
        USART_SendData(USART3, byte);        //ͨ���⺯��  ��������
        while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);  
        //�ȴ�������ɡ�   ��� USART_FLAG_TC �Ƿ���1��    //���⺯�� P359 ����
 }

void Uart3SendBuf(char *buf, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)Uart3SendByte(*buf++);
}
void Uart3SendStr(char *str)
{
	u16 i,len;
	len = strlen(str);
	for(i=0; i<len; i++)Uart3SendByte(*str++);
}

