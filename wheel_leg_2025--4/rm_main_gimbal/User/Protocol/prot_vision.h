#ifndef __PROT_VISION_H
#define __PROT_VISION_H

#include "stdint.h"
#include "math.h"


#define VISION_DATA_LEN 44
#define VISION_SP_RX_DATA_LEN 29
#define VISION_SP_TX_DATA_LEN 44

#define VISION_OFFLINE_TIMEOUT_MS 300

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

#pragma pack(1)
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
    uint32_t last_rx_tick;
    union
    {
        uint8_t buff[VISION_DATA_LEN];
         struct
        {
            float yaw;
            float pit;
            float yaw_vel;
            float yaw_acc;
            float pit_vel;
            float pit_acc;
            float shoot_yaw_tole;
			float shoot_pit_tole;
            float fire;
            float fire_rf;
            uint8_t empty;
            uint8_t cnt : 6;
            uint8_t ist_flag :1;
            uint8_t aim_flag :1;
			uint8_t trace_id;
            uint8_t eof;
        } data;
    } rx[2];
    union
    {
        uint8_t buff[23];
         struct
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
    
    union
    {
        uint8_t buff[8];
         struct
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
    struct
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

typedef enum
{
    VISION_SP_MODE_IDLE = 0,
    VISION_SP_MODE_AUTO_AIM = 1,
    VISION_SP_MODE_SMALL_BUFF = 2,
    VISION_SP_MODE_BIG_BUFF = 3
} vision_sp_mode_e;

typedef enum
{
    VISION_SP_RX_MODE_NO_CONTROL = 0,
    VISION_SP_RX_MODE_CONTROL = 1,
    VISION_SP_RX_MODE_FIRE = 2
} vision_sp_rx_mode_e;

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    uint8_t yaw[4];        // rad
    uint8_t yaw_vel[4];    // rad/s
    uint8_t yaw_acc[4];    // rad/s^2
    uint8_t pitch[4];      // rad
    uint8_t pitch_vel[4];  // rad/s
    uint8_t pitch_acc[4];  // rad/s^2
    uint8_t crc16[2];
} vision_rx_msg_sp_t;

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    uint8_t q[4][4];        // w, x, y, z
    uint8_t yaw[4];         // rad
    uint8_t yaw_vel[4];     // rad/s
    uint8_t pitch[4];       // rad
    uint8_t pitch_vel[4];   // rad/s
    uint8_t bullet_speed[4];// m/s
    uint8_t bullet_count[2];
    uint8_t camp;
    uint8_t crc16[2];
} vision_tx_msg_sp_t;

#pragma pack()

extern vision_t vision;
extern vision_tx_msg_t vision_tx_msg;
extern vision_rx_msg_sp_t vision_rx_msg_sp;
extern vision_tx_msg_sp_t vision_tx_msg_sp;
extern uint32_t shoot_Cnt;
void vision_get_data(uint8_t *data, uint32_t len);
void vision_output_data(void);
void vision_output_data_sp(void);
uint8_t vision_check_offline(void);
void vision_gimbal_get_data(vision_t * vision, uint32_t id, uint8_t *data);


#endif
