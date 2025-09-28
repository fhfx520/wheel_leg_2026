#include "BMI088_Read.h"
#include "bsp_JY901.h"
#include "bsp_imu.h"
#include "KalmanFilter.h"
#include "pid.h"
#include "bsp_bmi088.h"
#include "tim.h"
IMU_FLOAT_DATA_T imu_output_data;
uint8_t palstance_tx_data[12];	// [0,3]:X轴角速度 ；[4,7]:Y轴角速度 ；[8,11]:Z轴角速度
uint8_t accelerat_tx_data[12];	// [0,3]:X轴加速度 ；[4,7]:Y轴加速度 ；[8,11]:Z轴加速度
float gyro_z[500];
float current;

void BMI_Read(void)
{
    BMI088_original_data_read();    //原始数据读取
    BMI088_temp_data_read();        //BMI088传感器温度
    BMI088_Filter();                //滤波处理

    current = pid_calc(&pid_temperature, Temperature, 40);
    if(current<0)
        current = 0;
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, current);
    
    Float2Byte(&imu_output_data.Gyro.X, palstance_tx_data, 0);//底盘IMU
    Float2Byte(&imu_output_data.Gyro.Y, palstance_tx_data, 4);
    Float2Byte(&imu_output_data.Gyro.Z, palstance_tx_data, 8);

//    Float2Byte(&INS.MotionAccel_n[Y_axis], accelerat_tx_data, 0);
//    Float2Byte(&INS.MotionAccel_n[X_axis], accelerat_tx_data, 4);
//    Float2Byte(&INS.MotionAccel_n[Z_axis], accelerat_tx_data, 8);
    
//    float temp;
//    temp = -imu_output_data.Gyro.X;
//    Float2Byte(&temp, palstance_tx_data, 0);
//    temp = -imu_output_data.Gyro.Y;
//    Float2Byte(&temp, palstance_tx_data, 4);
//    Float2Byte(&imu_output_data.Gyro.Z, palstance_tx_data, 8);
    
    Float2Byte(&INS.MotionAccel_n[Y_axis], accelerat_tx_data, 0);
    Float2Byte(&INS.MotionAccel_n[X_axis], accelerat_tx_data, 4);
    Float2Byte(&INS.MotionAccel_n[Z_axis], accelerat_tx_data, 8);
}
