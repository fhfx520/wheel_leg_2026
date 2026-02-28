#ifndef __BOARD_COMM_H
#define __BOARD_COMM_H
#include <stdint.h>
#include "container_bus.h"


#define FDCAN_BOARD_DATA_LEN 		 64
#define FDCAN_CHA_TO_GIMBAL_ID		 0x001
#define FDCAN_GIMBAL_TO_CHA_ID		 0x011


#define TAG_KEYBOARD_DATA 	0x10
#define TAG_GIMBAL_DATA 	0x20
#define TAG_VISION_DATA		0x30

#define TAG_TX_RC_DATA 		0x40
#define TAG_TX_KB_DATA 		0x50
#define TAG_TX_IMU_DATA 	0x60
#define TAG_TX_JUDGE_DATA 	0x70
#define TAG_TX_VISION_DATA 	0x80
#define TAG_TX_GIMBAL_DATA 	0x90

#define TAG_TX_ALL_DATA		0x100

#pragma pack(1)

typedef struct //7bytes
{
	int16_t ch1;
	int16_t ch2;
	uint8_t sw1;
	uint8_t sw2;
	uint8_t rc_init_status;
} rc_data_t;//遥控器值

typedef struct //10bytes
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t l;
	uint8_t r;
	uint16_t key_code;
} kb_data_t;//跟随遥控器一起的键鼠指令

typedef struct //8bytes
{
	float pit;
	float rol;
} imu_data_t;//底盘陀螺仪数据

typedef struct //1bytes
{
	uint8_t camp;
} judge_data_t;//裁判系统数据

typedef struct //9bytes
{
	uint8_t vision_ID;
	float vision_bias_time;
	float shoot_speed;
} vision_tx_data_t;

typedef struct //16bytes
{
	float feedback_alpha_speed_input;
	float feedback_beta_speed_input;
	//yaw轴6020数据转发
	uint8_t yaw_raw_data[8];
} gimbal_tx_data_t;//云台控制所需数据

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
		struct
		{
			rc_data_t rc_data;
			kb_data_t kb_data;
			imu_data_t imu_data;
			judge_data_t judge_data;
			vision_tx_data_t vision_data;
			gimbal_tx_data_t gimbal_data;
			//保留
			uint8_t reserved[FDCAN_BOARD_DATA_LEN - (sizeof(rc_data_t) + sizeof(kb_data_t) \
							+ sizeof(imu_data_t) + sizeof(judge_data_t) + sizeof(vision_tx_data_t) + sizeof(gimbal_tx_data_t))];
		} e;
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
void board_comm_container_set(void);
void fdcan_board_comm_get(uint32_t id,uint8_t *pdata);

#endif
