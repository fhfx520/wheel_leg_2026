#ifndef __PROT_VTM_H
#define __PROT_VTM_H
#include "usart.h"
#include "stdint.h"
#define VTM_DATA_LEN 21

#pragma pack(1)
typedef __PACKED_STRUCT
{
	__PACKED_STRUCT
	{
		uint16_t channel_0 : 11;
		uint16_t channel_1 : 11;
		uint16_t channel_2 : 11;
		uint16_t channel_3 : 11;
		uint8_t sw		   :  2;
		uint8_t pause_key  :  1;
		uint8_t left_key   :  1;
		uint8_t right_key  :  1;
		uint16_t dial	   : 11;
		uint8_t trigger	   :  1;
		uint8_t empty	   :  3;
	} remote_data;
	__PACKED_STRUCT
	{
		int16_t mouse_x;
		int16_t mouse_y;
		int16_t mouse_z;
		uint8_t mouse_l : 2;
		uint8_t mouse_r : 2;
		uint8_t mouse_m : 2;
		uint8_t empty   : 2;
	} mouse_data;
	__packed union
	{
		uint16_t key_code;
		__PACKED_STRUCT
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;//16¸ö¼üÎ»
		} kb_data;
	} kb;
} vtm_data_t;

typedef struct
{
	vtm_data_t vtm_data;
	uint8_t online;
} vtm_t;
#pragma pack()

void vtm_get_data(uint8_t *data);
uint8_t vtm_check_offline(void);
extern vtm_t vtm;

#endif
