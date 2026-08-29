#ifndef __DRV_DM_MOTOR_H
#define __DRV_DM_MOTOR_H

#include "can_comm.h"
#include "data_list.h"

#define PIT_MOTOR_CMD_ID	0x05
#define PIT_MOTOR_REC_ID	0x15

#define YAW_MOTOR_CMD_ID	0x07
#define YAW_MOTOR_REC_ID	0x17

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ENABLE_MODE     0x03

#define P_MIN -12.56637f   // Radians
#define P_MAX 12.56637f
#define V_MIN -30.0f    // Rad/s
#define V_MAX 30.0f
#define KP_MIN 0.0f     // N*m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N*m/rad/s
#define KD_MAX 5.0f
#define T_MIN -10.0f    // N*m 18
#define T_MAX 10.0f
typedef struct
{
    list_t list;
    //电机参数
    can_channel_e can_channel;
    uint32_t can_id;
    uint32_t mst_id;
    uint32_t send_cnt, receive_cnt;
    float err_percent;
    uint8_t online;
    //安装角度补偿
    float zero_point;
    //控制数据
    float p, v, kp, kd, t;
    //反馈数据
	uint8_t state;
    uint8_t err_state;
    float position, velocity, torque;   //rad rad/s N*m
} dm_motor_t;

extern dm_motor_t joint_motor[4];
extern dm_motor_t trigger_motor_2325;			//新拨盘电机dm2325
//extern dm_motor_t pit_motor;					//新云台dm4310		1111测试架暂时注释
//extern dm_motor_t yaw_motor;

void dm_motor_init(dm_motor_t *motor, can_channel_e can_channel, uint32_t id, float zero_point, uint32_t mst_id);
void dm_motor_set_control_para(dm_motor_t *motor, float p, float v, float kp, float kd, float t);
void dm_motor_set_control_cmd(dm_motor_t *motor, uint8_t cmd);
void dm_motor_get_data(uint8_t id, uint8_t *data);
void dm_motor_output_data(void);
void dm_motor_output_enable_data(dm_motor_t *motor);
void dm_motor_output_single_data(dm_motor_t *motor);
void dm_error_opetate(dm_motor_t *motor, uint8_t* data);


void dm_motor_get_single_data(dm_motor_t *motor, uint8_t *data);

uint8_t dm_motor_check_offline(void);

#endif

