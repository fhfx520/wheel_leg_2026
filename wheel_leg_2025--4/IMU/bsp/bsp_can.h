#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "BMI088_Read.h"
#include "stm32f3xx_hal.h" 
#include "can.h"
#include "IMU_AHRSupdate.h"

void can_device_init(void);

void send_pit_message(void);
void send_yaw_message(void);
void send_rol_message(void);
void send_acc_message(void);

#endif
