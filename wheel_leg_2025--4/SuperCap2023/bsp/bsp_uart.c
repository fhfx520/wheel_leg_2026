#include "bsp_uart.h"
#include "stdlib.h"
#include "string.h"
#include "power_ctrl_task.h"
#include "bsp_judge.h"

uint8_t	dma_judge_rec_buf[DMA_JUDGE_LEN];


 /**
  * @brief  各个串口功能函数
  * @param 	UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	/* 裁判系统串口 */
	if(huart->Instance == USART1)
	{
		judge_data_handler(dma_judge_rec_buf);
		HAL_UART_Receive_DMA(&huart1, dma_judge_rec_buf, DMA_JUDGE_LEN);
	}
}


/**
  * @brief 串口空闲中断（需在it.c中每个串口的中断中调用该函数）
  * @param UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
		HAL_UART_DMAStop(huart);															//停止本次DMA运输
		USER_UART_IDLECallback(huart);                        //调用串口功能函数
	}
}


/**
* @brief 串口初始化:使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
void USER_UART_Init(void)
{
	__HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_rec_buf, DMA_JUDGE_LEN);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_FLUSH_DRREGISTER(huart);  //读DR寄存器，就可以清除ORE错误标志位
    }

		HAL_UART_DMAStop(huart);		
		HAL_UART_Receive_DMA(&huart1, dma_judge_rec_buf, DMA_JUDGE_LEN);
}

