/**
	* @file bsp_delay.c
	* @version 1.0
	* @date 2019.11.28
  *
  * @brief  用户自定义延时函数
  *
  *	@author YY
  *
  */
	
#include "bsp_delay.h"
	
void delay_us(uint16_t nus)
{
	uint16_t differ=0xffff-nus-5;	//设定定时器计数器起始值

	__HAL_TIM_SET_COUNTER(&htim16,differ);

	HAL_TIM_Base_Start(&htim16);		

  while(differ<0xffff-6)		
  {
    differ=__HAL_TIM_GET_COUNTER(&htim16);
  }
	
  HAL_TIM_Base_Stop(&htim16);
	
}




