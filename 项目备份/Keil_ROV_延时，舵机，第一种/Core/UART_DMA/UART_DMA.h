/*
 * UART_DMA.h
 *
 *  Created on: Mar 14, 2021
 *      Author: Royic
 */

#ifndef UART_DMA_UART_DMA_H_
#define UART_DMA_UART_DMA_H_

#include "main.h"

extern UART_HandleTypeDef huart1;	//修改为所用串口
#define USB_Huart huart1			//修改为所用串口

#define UART_RX_BUF_SIZE 128

#define UART_RXTX_Switch 1			//串口回显开关

/*
要在Cube中开串口全局中断和收发DMA
 */

extern uint8_t RxBuffer[UART_RX_BUF_SIZE];
extern uint8_t TxBuffer[UART_RX_BUF_SIZE];
extern uint8_t TxLen;

void USB_DMA_printf(const char *format,...);			//printf DMA方式
void USB_printf(const char *format,...);				//printf 普通方式
uint8_t UartTxData(UART_HandleTypeDef *huart, uint8_t *buf, const uint32_t len);
uint8_t StartUartRxDMA();								//接收DMA初始化
uint8_t StartUartTxDMA();								//不需要自己调用
void ProcessData();										//在里面添加数据处理函数
void HAL_UART_IdleCallback(UART_HandleTypeDef *huart);	//到USARTx_IRQHandler中添加

#endif /* UART_DMA_UART_DMA_H_ */
