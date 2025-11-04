
#include "prot_tfmini.h"
#include "wlr.h"

uint16_t tfmin_distance[1];

/**
  * @brief  TFminiPlus 数据解算
  * @param  uint8_t *buff
  * @retval 距离信息cm
  */
void vTfGetData(uint8_t *buff,TF_Distance_e TF_Distance)
{
	uint16_t distance,strength;
	if(buff[0]==0x59&&buff[1]==0x59) //校验帧头
	{
		distance=buff[3]<<8 | buff[2];//合并高低八位
		strength=buff[5]<<8 | buff[4];//合并高低八位
		if(strength>100&&strength!=65535) //在强度范围内才为有效数据
		{
			tfmin_distance[TF_Distance] = distance ;  //手动乘10，单位化为 mm，个位无意义
			wlr.front_dis_fdb  = tfmin_distance[TF_Distance]  * 10 ;
		}
	}
}

