#include "bsp_flash.h"
#include "data_processing.h"
#include "string.h"
void flash_erase_address(FLASH_EraseInitTypeDef *flash_erase,uint32_t address, uint16_t len)
{
	        uint32_t error;
          flash_erase->PageAddress = address;
	        flash_erase->TypeErase = FLASH_TYPEERASE_PAGES;
	        flash_erase->NbPages = len;
	        HAL_FLASH_Unlock();
          HAL_FLASHEx_Erase(flash_erase, &error);
          HAL_FLASH_Lock();
}
int8_t flash_write_single_address(FLASH_EraseInitTypeDef *flash,uint32_t start_address, float data)
{
         uint8_t data_msg[4];
	       Float2Byte(&data,data_msg,0);
         HAL_FLASH_Unlock();
	    for(int i=0;i<4;i++)
	       HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,start_address+i*2,data_msg[i]);
      HAL_FLASH_Lock();
    return 1;
}
void flash_read(uint32_t address,float* read_data )
{
	    uint8_t read_msg[4];
      for(int i=0;i<4;i++)
        read_msg[i] = *(__IO uint16_t*)(address+i*2);
      memcpy(read_data,read_msg,4);
}
