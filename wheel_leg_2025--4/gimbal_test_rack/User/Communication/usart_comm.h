#ifndef __USART_COMM_H
#define __USART_COMM_H

#include "stm32h7xx.h"
#include "usart.h"

//串口定义

//4号
//#define DEBUG_HUART	huart6//debug调试
//#define JUDGE_HUART huart2//裁判系统
//#define	DBUS_HUART	huart1//1111 遥控器
//#define VTM_HUART huart6

//1111测试架
#define DEBUG_HUART	huart6//debug调试
//#define NEW_IMU_HUART huart6//新陀螺仪暂时串口，原本裁判系统
#define	DBUS_HUART	huart1//1111 遥控器

void usart_comm_init(void);
void usart_user_handler(UART_HandleTypeDef *huart);

#endif
