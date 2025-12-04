#ifndef __ICM42688_DRIVER_H__
#define __ICM42688_DRIVER_H__

#include "stdint.h"
#include "main.h"



#define G_ACC  9.81f


#define LSB_ACC_16G		0.0047856934f
#define LSB_ACC_8G		0.0023928467f
#define LSB_ACC_4G		0.0011964233f
#define LSB_ACC_2G		0.00059821167f

/*Turn Into Radian*/
#define LSB_GYRO_2000_R	0.0010652644f
#define LSB_GYRO_1000_R	0.00053263222f
#define LSB_GYRO_500_R	0.00026631611f
#define LSB_GYRO_250_R	0.00013315805f
#define LSB_GYRO_125D_R	0.000066579027f

typedef struct
{
    float Accel[3];
		float Accel_raw[3];
    float Gyro[3];
		float Gyro_raw[3];
    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
	
	  float Calidata[5];
	
} IMU_Data_t;


float bsp_Icm42688GetAres(uint8_t Ascale);

float bsp_Icm42688GetGres(uint8_t Gscale);

void bsp_IcmGetRawData(IMU_Data_t *ICM42688);

void ICM42688P_ConvertToPhysical(IMU_Data_t *ICM42688);

int8_t bsp_IcmGetTemperature(int16_t* pTemp);

int16_t ICM42688_init(void);

extern uint8_t init_flag;
extern IMU_Data_t IMU_Data;
extern float accel_scale;
extern float gyro_scale;
extern int16_t temp;
#endif
