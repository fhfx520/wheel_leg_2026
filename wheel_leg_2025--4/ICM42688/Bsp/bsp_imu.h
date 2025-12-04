#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "main.h"
#include "math.h"
#include "ICM42688_driver.h"


#define GxOFFSET 0.00247530174f
#define GyOFFSET 0.000393082853f
#define GzOFFSET 0.000393082853f

typedef enum{
   hight_temperature,
	 normal,
	temperature_error
}imu_mode_e;



#define X_axis 0
#define Y_axis 1
#define Z_axis 2
#define Accel_c 3
#define Temp_Cali 4

typedef enum{
   Calibration_error_mode,
	 Calibration_successful_mode
}bias_gyro_mode_e;

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

/***************/
extern imu_mode_e imu_mode;
extern uint8_t califlag;//初始化标志位
extern bias_gyro_mode_e bias_gyro_mode;

void Calibrate_MPU_Offset(IMU_Data_t *ICM42688);

void IMU_AHRS_Calcu_task(void);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void ComplementaryFilter(const IMU_Data_t *ICM42688 ,float alpha, float dt);
void calculate_euler_angles(IMU_Data_t *ICM42688,float alpha,float dt);
#endif


