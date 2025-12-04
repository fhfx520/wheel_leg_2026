#include "ICM42688_Middleware.h"
#include "main.h"
#include "spi.h"

SPI_HandleTypeDef *ICM42688_SPI;


uint8_t ICM42688_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
//    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 5);
		HAL_SPI_TransmitReceive_DMA(&hspi1, &txdata, &rx_data, 1);
    return rx_data;
}
