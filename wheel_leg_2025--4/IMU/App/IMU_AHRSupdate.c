#include "IMU_AHRSupdate.h"

extern IMU_FLOAT_DATA_T imu_real_data;

uint8_t angle_tx_data[12];
uint8_t offset_time=0;

float flag_quaternion_intilized;

void IMU_AHRSupdate_task(void)
{
    IMU_Values_Convert();   //原始数据换算
    IMU_AHRS_Calcu();       //姿态角解算
    imu_real_data.yaw+= 180.0f;

    Float2Byte(&imu_real_data.pitch, angle_tx_data, 0);//底盘IMU
    Float2Byte(&imu_real_data.roll, angle_tx_data, 4);
    Float2Byte(&imu_real_data.yaw, angle_tx_data, 8);
    
//    float temp;
//    temp = -imu_real_data.pitch;
//    Float2Byte(&temp, angle_tx_data, 0);
//    temp = -imu_real_data.roll;
//    Float2Byte(&temp, angle_tx_data, 4);
//    Float2Byte(&imu_real_data.yaw, angle_tx_data, 8);
}
