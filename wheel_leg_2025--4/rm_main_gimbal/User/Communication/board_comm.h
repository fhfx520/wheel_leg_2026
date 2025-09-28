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

extern board_comm_t board_comm;
void board_comm_get_data(uint32_t id, uint8_t *data);
void board_comm_send_data(void);

#endif
