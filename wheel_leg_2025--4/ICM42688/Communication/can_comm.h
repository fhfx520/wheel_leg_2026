#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32g4xx.h"

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


}imu_msg_t;	

void can_comm_init(void);
void can_std_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

extern imu_msg_t imu_msg_send;

#endif
