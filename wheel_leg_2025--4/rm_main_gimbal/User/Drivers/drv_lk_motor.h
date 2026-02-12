#ifndef __DRV_LK_MOTOR_H
#define __DRV_LK_MOTOR_H

#include "can_comm.h"
#include "data_list.h"

//ID
#define LK_GIMBAL_ID					0x142
#define LK_GIMBAL_RECEIVE_ID			0x142

#define LK_GIMBAL_OFFSET				0x00
typedef enum
{
	MG8016 = 0,
	MF5015 = 1,
}lk_motor_type_e;
typedef enum
{
	CURRENT_MODE = 0xA1,
	READ_ERR	 = 0x9A,
	CLEAR_ERR	 = 0x9B,
	START_MOTOR  = 0x88,
	CLOSE_MOTOR	 = 0x80,
}lk_motor_cmd_e;
typedef struct
{
	list_t list;
	//电机参数
	uint8_t motor_type;
	can_channel_e can_channel;	
	uint32_t can_id,mst_id;
	uint32_t send_cnt, receive_cnt;
	float err_percent;
	//安装角度补偿
	float zero_point;	
	//电机输出轴分辨率
	uint8_t resoul_bits;
	//分辨率对应的最大编码器值
	uint32_t max_ecd;
	//发送力矩电流
	int16_t tx_current;
	//反馈初始数据
	int8_t temp;				//温度
	int16_t rx_current;			//反馈电流
	int16_t speed;				// °/s
	uint16_t ecd;				//反馈编码器值
	//处理后的位置速度数据
	float last_p, p, v, t;		
	uint8_t err_code;
	uint8_t online;
	//有传动比的情况下，反馈数据换算到输出轴
	float p_tmp,real_p,real_v;
}lk_motor_t;

void lk_motor_init(lk_motor_t *motor, can_channel_e can_channel, uint32_t id, float zero_point, uint32_t mst_id, uint8_t resoul_bits);
void lk_set_cmd(lk_motor_t* motor,lk_motor_cmd_e cmd);
void lk_set_current(lk_motor_t* motor);
void lk_get_single_data(lk_motor_t *motor,uint8_t *data);

#endif
