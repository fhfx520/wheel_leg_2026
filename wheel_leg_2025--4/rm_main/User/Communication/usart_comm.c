#include "usart_comm.h"
#include "prot_dr16.h"
#include "prot_judge.h"
#include "string.h"
#include "data_log.h"
#include "prot_tfmini.h"
#include "prot_hipnuc.h"
#include "prot_ms53l2m.h"
#include "prot_ms53l0m.h"
#include "prot_tof.h"
#define DEBUG_DATA_LEN 10
#define JUDGE_DATA_LEN 150
#define TFMINIPLUS_BUFF_SIZE 50
#define TOF_BUFF_SIZE 11

uint8_t dr16_dma_rx_buf[DR16_DATA_LEN];
uint8_t judge_data_rx_buf[JUDGE_DATA_LEN];
uint8_t debug_dma_rx_buf[DEBUG_DATA_LEN];
//uint8_t TFminiPlusBuffArray_Front_Left[TFMINIPLUS_BUFF_SIZE];
//uint8_t TFminiPlusBuffArray_Front_Right[TFMINIPLUS_BUFF_SIZE];
uint8_t TOFBuffArray_LEFT[TOF_BUFF_SIZE];
uint8_t TOFBuffArray_RIGHT[TOF_BUFF_SIZE];
//uint8_t Hipnuc_buff[82];
uint32_t ccnct;
/*
 * @brief  串口初始化，开启空闲中断并开始DMA接收数据
 * @retval void
 */
void usart_comm_init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dr16_dma_rx_buf, DR16_DATA_LEN);
    
    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, judge_data_rx_buf, JUDGE_DATA_LEN);
    judge_init(&JUDGE_HUART);

//    __HAL_UART_CLEAR_IDLEFLAG(&DEBUG_HUART);
//    __HAL_UART_ENABLE_IT(&DEBUG_HUART, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&DEBUG_HUART, debug_dma_rx_buf, DEBUG_DATA_LEN);
    log_init(&DEBUG_HUART);

    __HAL_UART_CLEAR_IDLEFLAG(&TOF_LEFT_HUART);
    __HAL_UART_ENABLE_IT(&TOF_LEFT_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&TOF_LEFT_HUART, TOFBuffArray_LEFT, TOF_BUFF_SIZE);
	
	__HAL_UART_CLEAR_IDLEFLAG(&TOF_RIGHT_HUART);
    __HAL_UART_ENABLE_IT(&TOF_RIGHT_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&TOF_RIGHT_HUART, TOFBuffArray_RIGHT, TOF_BUFF_SIZE);
}

/*
 * @brief  串口中断，添加各串口的数据接收函数
 * @retval void
 */
void usart_user_handler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_AbortReceive(huart);
        if (huart == &DBUS_HUART) {
            dr16_get_data(&rc, dr16_dma_rx_buf);
            HAL_UART_Receive_DMA(huart, dr16_dma_rx_buf, DR16_DATA_LEN);
        } else if (huart == &JUDGE_HUART) {
            judge_get_data(judge_data_rx_buf);
            HAL_UART_Receive_DMA(huart, judge_data_rx_buf, JUDGE_DATA_LEN);
        } else if (huart == &TOF_RIGHT_HUART) { 
			tof_get_data(TOFBuffArray_RIGHT, TOF_BUFF_SIZE,1);
            HAL_UART_Receive_DMA(huart, TOFBuffArray_RIGHT, TOF_BUFF_SIZE);
		} 
        else if (huart == &TOF_LEFT_HUART) {
            tof_get_data(TOFBuffArray_LEFT, TOF_BUFF_SIZE,0);
            HAL_UART_Receive_DMA(huart, TOFBuffArray_LEFT, TOF_BUFF_SIZE);
			ccnct++;
        }
    }
}
