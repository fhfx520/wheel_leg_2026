/** 
  * @file     bsp_uart.c
  * @version  v2.0
  * @date     2019.11.18
	*
  * @brief    串口驱动
	*
  *	@author   YY
  *
  */
	
#include "bsp_uart.h"
#include "bsp_JY901.h"
uint8_t JY901_receive_buff[JY901_BUFLEN];
/**
* @brief 使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
void user_uart_init()
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);
}


/**
* @brief 串口空闲中断
* @param UART_HandleTypeDef *huart
* @retval 无
*/
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     	//清除空闲中断标志（否则会一直不断进入中断）
		HAL_UART_DMAStop(huart);
		USAR_UART_IDLECallback(huart);                       	  //调用中断处理函数
	}
}


/**
* @brief 串口空闲中断回调函数
* @param 串口句柄
* @retval 无
*/
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance== USART1 )
	{
		JY901_original_data_read(JY901_receive_buff); 			//JY901串口读取原始数据
	}
	memset(JY901_receive_buff,0,sizeof(JY901_receive_buff));                    //清零接收缓冲区
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)JY901_receive_buff, JY901_BUFLEN);  //重启开始DMA传输
}


