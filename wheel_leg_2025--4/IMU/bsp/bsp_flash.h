#ifndef BSH_FLASH_H
#define BSH_FLASH_H
#include "main.h"
#define BIAS_GYRO_X_ADDRESS 0x08007890
#define BIAS_GYRO_Y_ADDRESS 0x0800789c
#define BIAS_GYRO_Z_ADDRESS 0x080078A8
#define BIAS_INIT_TEMPTURE  0x080078BA
#define BIAS_ACCEL_X_ADDRESS 0x080078C8
#define BIAS_ACCEL_Y_ADDRESS 0x080078D6
#define BIAS_ACCEL_Z_ADDRESS 0x080078E4
void flash_erase_address(FLASH_EraseInitTypeDef *flash_erase,uint32_t address, uint16_t len);
int8_t flash_write_single_address(FLASH_EraseInitTypeDef *flash,uint32_t start_address, float data);
void flash_read(uint32_t address,float* read_data );
#endif
