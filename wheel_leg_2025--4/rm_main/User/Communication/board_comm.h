#ifndef __BOARD_COMM_H
#define __BOARD_COMM_H
#include <stdint.h>
#include "container_bus.h"


#define FDCAN_BOARD_DATA_LEN 		 64
#define FDCAN_CHA_TO_GIMBAL_ID		 0x001
#define FDCAN_GIMBAL_TO_CHA_ID		 0x011


#define TAG_KEYBOARD_DATA 0x10
#define TAG_GIMBAL_DATA 0x20
#define TAG_VISION_DATA 0x30


#pragma pack(1)
typedef struct
{
	union
    {
		//64bytes
        uint8_t buff[FDCAN_BOARD_DATA_LEN];
		struct //10bytes
        {
			struct
			{
				int16_t mouse_x;
				int16_t mouse_y;
				int16_t mouse_z;
				struct{
					uint8_t mouse_l : 2;
					uint8_t mouse_r : 2;
					uint8_t mouse_m : 2;
					uint8_t empty   : 2;
				} __attribute__((packed));
			} mouse_data;
			uint16_t key_code;
			uint8_t online;
		} data_keyboard;//图传链路键鼠数据
		struct //3bytes
		{
			int16_t yaw_output;
			uint8_t gimbal_start_up;
		} gimbal_data;//云台下发数据
		struct //3bytes
		{
			uint8_t vision_enanle;
			uint8_t vision_trace_id;
			uint8_t vision_online;
		}vision_data;//视觉数据
		//保留
		uint8_t reserved[FDCAN_BOARD_DATA_LEN - 16];
    } rx_msg;
	
	union
	{
		//64bytes
        uint8_t buff[FDCAN_BOARD_DATA_LEN];
	} tx_msg;
}fdcan_board_comm_t;

typedef struct
{
	struct
	{
		int16_t mouse_x;
		int16_t mouse_y;
		int16_t mouse_z;
		struct{
			uint8_t mouse_l : 2;
			uint8_t mouse_r : 2;
			uint8_t mouse_m : 2;
			uint8_t empty   : 2;
		} __attribute__((packed));
	} mouse_data;
	uint16_t key_code;
	uint8_t online;
} data_keyboard_t;//图传链路键鼠数据

typedef struct 
{
	int16_t yaw_output;
	uint8_t gimbal_start_up;
} gimbal_data_t;//云台下发数据

typedef struct 
{
	uint8_t vision_enanle;
	uint8_t vision_trace_id;
	uint8_t vision_online;
}vision_data_t;//视觉数据





#pragma pack()


extern fdcan_board_comm_t fdcan_board_comm;
void fdcan_board_comm_send(void);
void fdcan_board_comm_get(uint32_t id,uint8_t *pdata);

#endif
