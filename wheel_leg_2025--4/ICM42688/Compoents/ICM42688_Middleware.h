#ifndef __ICM42688_MIDDLEWARE_H
#define __ICM42688_MIDDLEWARE_H

#include "main.h"

uint8_t ICM42688_read_write_byte(uint8_t reg);

extern SPI_HandleTypeDef *ICM42688_SPI;


#endif


