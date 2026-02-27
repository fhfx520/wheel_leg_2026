#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32g4xx.h"

#define IMU_CHA_ID 0x015
#define IMU_GIM_ID 0X016

typedef struct{
    union
    {
        uint8_t array[8];
        struct
        {
            float pit;
            float wy;
        }e;
    } pit_msg;//pit
    union
    {
        uint8_t array[8];
        struct
        {
            float yaw;
            float wz;
        } e;
    } yaw_msg;//yaw
	union
    {
        uint8_t array[8];
        struct
        {
            float rol;
            float wx;
        } e;
    } rol_msg;//roll
	union
    {
		 uint8_t array[8];
        struct
        {
            float wy;
            float wz;
        } e;
    } gim_w_msg;//
	union
    {
		 uint8_t array[8];
        struct
        {
            float pit;
            float yaw;
        } e;
    } gim_angle_msg;//
	union
    {
		 uint8_t array[8];
		struct
		{
			float ay;
			float az;
		} e;
	} cha_angle_msg;//
	union
	{
		uint8_t array[64];
		struct
		{
			struct //12bytes
			{
				float pit;
				float yaw;
				float rol;
			} angle;
			struct //12bytes
			{
				float wy;
				float wz;
				float wx;
			} gyro;
			struct //36bytes
			{
				//加速计测得世界系 az = g
				float ay;
				float az;
				float ax;
				//四元数转化到绝对系
				float n_ay;
				float n_az;
				float n_ax;
				//机体坐标系
				float b_ay;
				float b_az;
				float b_ax;
			} acc;
			//保留位 4bytes
			float r0;
		} e;
	} all_angle_msg;

}imu_msg_t;	

void can_comm_init(void);
void fdcan_comm_init(void);
void can_std_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

extern imu_msg_t imu_msg_send;

#endif
