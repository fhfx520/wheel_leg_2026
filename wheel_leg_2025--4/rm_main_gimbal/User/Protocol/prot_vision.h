#ifndef __PROT_VISION_H
#define __PROT_VISION_H

#include "stdint.h"
#include "math.h"


#define VISION_DATA_LEN 28

#define NAN_PROCESS(now, last)      \
    do {                            \
        if (isnan(now)) {           \
            (now) = (last);         \
        } else {                    \
            (last) = (now);         \
        }                           \
    } while (0)

typedef enum
{
    NORMAL = 0,
    TAIL_ERROR = 1,
    REPEAT_ERROR = 2
} vision_rx_status_e;

typedef enum
{
    UNAIMING = 0,
    AIMING = 1,
    FIRST_LOST = 2
} vision_aim_status_e;

typedef struct
{
    uint32_t rx_repeat_cnt;
    vision_rx_status_e rx_status;
    vision_aim_status_e aim_status;
    uint32_t new_frame_flag;
    float target_yaw_angle, target_pit_angle;
    float yaw_min_err;
		float pit_min_err;
    uint32_t shoot_enable;
		uint8_t trace_id;
    uint8_t online;
    __packed union
    {
        uint8_t buff[VISION_DATA_LEN];
         __packed struct
        {
            float yaw;
            float pit;
            float dis;
						float dis2;
            float fire;
            float pos;
            uint8_t empty;
            uint8_t cnt : 6;
            uint8_t ist_flag :1;
            uint8_t aim_flag :1;
						uint8_t trace_id;
            uint8_t eof;
        } data;
    } rx[2];
    __packed union
    {
        uint8_t buff[23];
         __packed struct
        {
            uint8_t sof;
            uint8_t imu_pit[4];
            uint8_t imu_yaw[4];
            uint8_t imu_pit_spd[4];
            uint8_t imu_yaw_spd[4];
					
            uint8_t vacancy :1;
            uint8_t camp :1;
            uint8_t aiming_mode :3;
            uint8_t shooter_speed :3;
						uint8_t trace_id;
						uint8_t shoot_speed[4];
            uint8_t empty;
            uint8_t eof1;
            uint8_t eof2;
					
            uint16_t empty1;
        } data;
    } tx;
    
    __packed union
    {
        uint8_t buff[8];
         __packed struct
        {
            uint8_t vision_shoot_enable;
            uint8_t start_up_flag;
            uint8_t empty2;
            uint8_t empty3;
            uint8_t empty4;
            uint8_t empty5;
            uint8_t empty6;
            uint8_t empty7;            
        } data;
    } rxt;
    
} vision_t;

typedef struct
{
    uint8_t sof; 
    uint8_t imu_pit[4];
    uint8_t imu_yaw[4];
    uint8_t imu_pit_spd[4];
    uint8_t imu_yaw_spd[4];
    __packed struct
    {
        uint8_t vacancy : 1;       
        uint8_t camp : 1;          
        uint8_t aiming_status : 3; 
        uint8_t shooter_speed : 3; 
    } mode_msg;
    
    uint8_t ID;
	uint8_t shoot_speed[4];
	
	uint8_t bias_time[4];
	
    uint8_t eof1; 
    uint8_t eof2; 
    
} vision_tx_msg_t;



extern vision_t vision;
extern vision_tx_msg_t vision_tx_msg;
void vision_get_data(uint8_t *data);
void vision_output_data(void);
uint8_t vision_check_offline(void);
void vision_gimbal_get_data(vision_t * vision, uint32_t id, uint8_t *data);


#endif
