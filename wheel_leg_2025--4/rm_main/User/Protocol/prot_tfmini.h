
#ifndef __TF_MINI_PLUS_H
#define __TF_MINI_PLUS_H

#include "stm32h7xx_hal.h"
#include "usart.h"


typedef enum {
    HEAD, 
}TF_Distance_e;

/*TFminiplus 处理后的数据*/
extern uint16_t tfmin_distance[1];

void vTfGetData(uint8_t *buff,TF_Distance_e TF_Distance);
#endif

