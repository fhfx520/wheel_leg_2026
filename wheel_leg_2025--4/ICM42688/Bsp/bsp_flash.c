#include "bsp_flash.h"
#include "data_processing.h"
#include "string.h"

FLASH_EraseInitTypeDef My_Flash;
void flash_erase_address(FLASH_EraseInitTypeDef *flash_erase,uint32_t address, uint16_t len)
{
			
	        uint32_t error;
          flash_erase->Page =  (address - FLASH_BASE) / FLASH_PAGE_SIZE;
	        flash_erase->TypeErase = FLASH_TYPEERASE_PAGES;
	        flash_erase->NbPages = len;
	        HAL_FLASH_Unlock();			
	
		

			// 擦除操作
			if (HAL_FLASHEx_Erase(flash_erase, &error) != HAL_OK)
			{
					// 擦除失败处理
					HAL_FLASH_Lock();
					Error_Handler();  // 用户可以自定义的错误处理函数
			}
			
          HAL_FLASH_Lock();
	

  

	
}

void Flash_Erase(uint32_t start_addr)
{
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;
    
    // 解锁Flash
    HAL_FLASH_Unlock();

    // 设置擦除类型和地址
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = (start_addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    eraseInitStruct.NbPages = 1;  // 擦除1页

    // 擦除操作
    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
    {
        // 擦除失败处理
        HAL_FLASH_Lock();
        Error_Handler();  // 用户可以自定义的错误处理函数
    }

    // 锁定Flash
    HAL_FLASH_Lock();
}


int8_t flash_write_single_address(FLASH_EraseInitTypeDef *flash,uint32_t start_address, float data)
{					
					uint64_t doubleword = 0;  // 用于存储64位的数据
	    uint32_t float_data;

    // 将 float 转换为 uint32_t
    memcpy(&float_data, &data, sizeof(float));
	    // 将 float 数据放入 doubleword 的前 4 字节，后 4 字节填充为 0
    doubleword = (uint64_t)float_data;

	
         HAL_FLASH_Unlock();
				
				    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_address, doubleword) != HAL_OK)
    {
        // 写入失败处理
        HAL_FLASH_Lock();
        Error_Handler();  // 用户可以自定义的错误处理函数
    }
		
				
        HAL_FLASH_Lock();
    return 1;
}
  





void flash_read(uint32_t address,float* read_data )
{ 
	    uint64_t read_msg;
			float real_data;
	        read_msg = *(__IO uint64_t*)(address);
//        read_msg = *(__IO uint64_t*)(address+i*8);
			
			memcpy(&real_data,&read_msg,4);
	
      memcpy(read_data,&real_data,4);
	
//		    uint8_t read_msg[4];
//      for(int i=0;i<4;i++)
//        read_msg[i] = *(__IO uint16_t*)(address+i*2);
//      memcpy(read_data,read_msg,4);
	
	
	   uint64_t doubleword = *(volatile uint64_t*)address;  // 从地址读取64位的数据
    uint32_t float_data = (uint32_t)(doubleword & 0xFFFFFFFF);  // 提取前4字节


    // 将 uint32_t 转换回 float
    memcpy(read_data, &float_data, sizeof(float));

}

void imu_flash_write(float *data_to_write,uint8_t num){

			 HAL_FLASH_Unlock();
		My_Flash.TypeErase = FLASH_TYPEERASE_PAGES;  //标明Flash执行页面只做擦除操作
  
   My_Flash.Page = BIAS_START_ADDRESS;  //声明要擦除的地址
   My_Flash.NbPages = 1;  

		Flash_Erase(BIAS_START_ADDRESS);
		    // 依次写入4个 float 数据到不同的地址
    for (uint8_t i = 0; i < num; i++)
    {
        flash_write_single_address(&My_Flash,BIAS_START_ADDRESS + i * 8, data_to_write[i]);
    }




}

void imu_flash_read(float *data_read,uint8_t num){

			 HAL_FLASH_Unlock();
		My_Flash.TypeErase = FLASH_TYPEERASE_PAGES;  //标明Flash执行页面只做擦除操作
  
   My_Flash.Page = BIAS_START_ADDRESS;  //声明要擦除的地址
   My_Flash.NbPages = 1;  

    // 从 Flash 中读取数据
    for (uint8_t i = 0; i < num; i++)
    {
         flash_read(BIAS_START_ADDRESS + i * 8,&data_read[i]);
    }

}
