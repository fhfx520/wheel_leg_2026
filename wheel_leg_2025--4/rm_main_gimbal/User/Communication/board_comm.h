#ifndef __BOARD_COMM_H
#define __BOARD_COMM_H

#include "stm32h7xx.h"


#define RC_MSG_ID 		 0x001
#define CHA_MSG_ID 		 0x002
#define VISION_MSG_ID 	 0x003
#define JUDGE_MSG_ID 	 0x005
#define VISION_UI_MSG_ID 0x006
#define SHOOT_MSG_ID 	 0x007
#define STABLE_MSG_ID 	 0x008
#define BOARD_DATA_LEN 	 8

#define FDCAN_BOARD_DATA_LEN 	 	 64
#define FDCAN_CHA_TO_GIMBAL_ID		 0x001
#define FDCAN_GIMBAL_TO_CHA_ID		 0x011

#define TAG_VTM_KEYBOARD_DATA 	0x10
#define TAG_GIMBAL_OUTPUT_DATA 	0x20
#define TAG_TRACE_VISION_DATA	0x30

#define TAG_DR16_RC_DATA 		0x40
#define TAG_DR16_KB_DATA 		0x50
#define TAG_CHA_IMU_DATA 		0x60
#define TAG_JUDGE_DATA 			0x70
#define TAG_SHOOT_VISION_DATA 	0x80
#define TAG_GIMBAL_CTRL_DATA 	0x90

#define TAG_TRANSMIT_DATA		0x100

typedef struct
{
    __packed union
    {
        uint8_t buff[BOARD_DATA_LEN];
         __packed struct
        {
			int16_t chl;
			int16_t ch2;
			uint8_t sw1;
			uint8_t sw2;
			uint8_t empty1;
			uint8_t empty2;
		} data_remote;
		__packed struct
        {
			int16_t x;
			int16_t y;
			uint8_t l;
			uint8_t r;
			uint16_t key_code;
		} data_keyboard;
    } rx_rc_msg;
		
	
	__packed union
    {
        uint8_t buff[BOARD_DATA_LEN];
         __packed struct
        {
			float  cha_pit;
			uint8_t ctrl_mode;
			uint8_t camp;
			uint8_t rc_init_status;
			uint8_t empty2;
        } data;
    } rx_cha_msg;
	
	
	__packed union
    {
        uint8_t buff[BOARD_DATA_LEN];
        __packed struct
        {
            uint8_t vision_ID;
            uint8_t empty1;
			uint8_t empty2;
			uint8_t empty3;
			uint8_t empty4;
			uint8_t empty5;
			uint8_t empty6;
			uint8_t empty7;
        } data;
    } rx_judge_msg;
	
	
	__packed union
    {
        uint8_t buff[BOARD_DATA_LEN];
        __packed struct
        {
			float vision_bias_time;
			float shoot_speed;
        } data;
    } rx_shoot_msg;
	
	
	__packed union
    {
        uint8_t buff[BOARD_DATA_LEN];
        __packed struct
        {
			float feedback_alpha_speed_input;
			float feedback_beta_speed_input;
        } data;
    } rx_stable_msg;
	
	
	/*            TX              */
		
	__packed union
    {
        uint8_t buff[BOARD_DATA_LEN];
         __packed struct
        {
			uint8_t  vision_enanle;
			uint8_t gimbal_start_up;
			uint8_t empty2;
			uint8_t empty3;
			uint8_t empty4;
			uint8_t empty5;
			uint8_t empty6;
			uint8_t empty7;
        } data;
    } tx_cha_msg;
				
	
	__packed union
    {
        uint8_t buff[BOARD_DATA_LEN];
         __packed struct
        {
			uint8_t vision_trace_id;
			uint8_t vision_online;
			uint8_t empty3;
			uint8_t empty4;
			uint8_t empty5;
			uint8_t empty6;
			uint8_t empty7;
        } data;
    } tx_vis_msg;
		
} board_comm_t;

#pragma pack(1)

typedef struct //8bytes
{
	int16_t ch1;
	int16_t ch2;
	uint8_t sw1;
	uint8_t sw2;
	uint8_t rc_init_status;
	uint8_t ctrl_mode;
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

typedef struct //11bytes
{
	uint8_t vision_ID;
	uint8_t energy_flag;
	uint8_t energy_state;
	float vision_bias_time;
	float shoot_speed;
} vision_data_t;

typedef struct //16bytes
{
	float feedback_alpha_speed_input;
	float feedback_beta_speed_input;
	//yaw轴6020数据转发
	uint8_t yaw_raw_data[8];
} gimbal_data_t;//云台控制所需数据

typedef struct
{
	union
    {
		//64bytes
        uint8_t buff[FDCAN_BOARD_DATA_LEN];
		struct
		{
			struct 
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
			} data_remote;
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
			uint8_t reserved[FDCAN_BOARD_DATA_LEN - 24];
		} e;
    } tx_msg;
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
			vision_data_t vision_data;
			gimbal_data_t gimbal_data;
			//保留
			uint8_t reserved[FDCAN_BOARD_DATA_LEN - (sizeof(rc_data_t) + sizeof(kb_data_t) \
							+ sizeof(imu_data_t) + sizeof(judge_data_t) + sizeof(vision_data_t) + sizeof(gimbal_data_t))];
		} e;
	} rx_msg;
}fdcan_board_comm_t;
#pragma pack()

extern board_comm_t board_comm;
extern fdcan_board_comm_t fdcan_board_comm;

extern rc_data_t rc_data_rec;
extern kb_data_t kb_data_rec;
extern imu_data_t imu_data_rec;
extern judge_data_t judge_data_rec;
extern vision_data_t vision_data_rec;
extern gimbal_data_t gimbal_data_rec;

void board_comm_get_data(uint32_t id, uint8_t *data);
void board_comm_send_data(void);

void fdcan_board_comm_send(void);
void fdcan_board_comm_get(uint32_t id, uint8_t *data);
uint8_t board_comm_check_offline(void);

#endif
