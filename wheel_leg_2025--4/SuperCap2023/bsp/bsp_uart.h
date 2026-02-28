#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define   JUDGE_HUART     huart1

//串口DMA接收缓冲区长度
#define  DMA_JUDGE_LEN		300 

void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);

#endif

